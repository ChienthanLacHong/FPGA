`timescale 1ns/1ps

module uart_frame_streamer_1bpp #(
    parameter integer CLK_FREQ_HZ  = 50000000,
    parameter integer BAUD         = 2000000,
    parameter integer WIDTH        = 640,
    parameter integer HEIGHT       = 480,
    parameter integer PAYLOAD_LEN  = (WIDTH*HEIGHT)/8,
    parameter integer ADDR_WIDTH   = 16,
    parameter integer HEADER_LEN   = 20
)(
    input  wire                  clk_uart,
    input  wire                  rst_n,

    input  wire                  frame_ready_vga,
    input  wire [31:0]           frame_id_vga,

    output reg                   consume_toggle_uart,

    output reg  [ADDR_WIDTH-1:0] rd_addr,
    input  wire [7:0]            rd_data,

    output wire                  uart_tx
);

    // -------- sync frame_ready into clk_uart
    reg [1:0] fr_sync;
    always @(posedge clk_uart or negedge rst_n) begin
        if (!rst_n) fr_sync <= 2'b00;
        else        fr_sync <= {fr_sync[0], frame_ready_vga};
    end
    wire frame_ready = fr_sync[1];

    reg [31:0] frame_id_lat;

    // -------- uart tx (1x per bit)
    reg        tx_start;
    reg [7:0]  tx_data;
    wire       tx_busy;
    wire       tx_done;

    uart_tx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD       (BAUD)
    ) u_tx (
        .clk     (clk_uart),
        .rst_n   (rst_n),
        .tx_start(tx_start),
        .data_in (tx_data),
        .tx_busy (tx_busy),
        .tx_done (tx_done),
        .tx_line (uart_tx)
    );

    // -------- header byte function
    function [7:0] header_byte;
        input [15:0] idx;
        begin
            case (idx)
                16'd0:  header_byte = 8'hA5;
                16'd1:  header_byte = 8'h5A;
                16'd2:  header_byte = 8'hF0;
                16'd3:  header_byte = 8'h0F;

                16'd4:  header_byte = frame_id_lat[7:0];
                16'd5:  header_byte = frame_id_lat[15:8];
                16'd6:  header_byte = frame_id_lat[23:16];
                16'd7:  header_byte = frame_id_lat[31:24];

                16'd8:  header_byte = 8'h80; // 640 LE
                16'd9:  header_byte = 8'h02;

                16'd10: header_byte = 8'hE0; // 480 LE
                16'd11: header_byte = 8'h01;

                16'd12: header_byte = 8'h00; // fmt=0
                16'd13: header_byte = 8'h00;

                // payload_len = 38400 = 0x00009600 LE
                16'd14: header_byte = 8'h00;
                16'd15: header_byte = 8'h96;
                16'd16: header_byte = 8'h00;
                16'd17: header_byte = 8'h00;

                16'd18: header_byte = 8'h00;
                16'd19: header_byte = 8'h00;

                default: header_byte = 8'h00;
            endcase
        end
    endfunction

    // -------- streamer FSM
    localparam ST_IDLE     = 0;
    localparam ST_HDR      = 1;
    localparam ST_RD_WAIT  = 2;
    localparam ST_PAY_SEND = 3;
    localparam ST_DONE     = 4;

    reg [2:0]  st;
    reg [15:0] hdr_i;
    reg [31:0] pay_i;

    always @(posedge clk_uart or negedge rst_n) begin
        if (!rst_n) begin
            st                  <= ST_IDLE;
            hdr_i               <= 0;
            pay_i               <= 0;
            rd_addr             <= 0;
            tx_start            <= 1'b0;
            tx_data             <= 8'h00;
            consume_toggle_uart <= 1'b0;
            frame_id_lat        <= 32'd0;
        end else begin
            tx_start <= 1'b0;

            case (st)
                ST_IDLE: begin
                    hdr_i   <= 0;
                    pay_i   <= 0;
                    rd_addr <= 0;
                    if (frame_ready && !tx_busy) begin
                        frame_id_lat <= frame_id_vga;
                        tx_data      <= header_byte(16'd0);
                        tx_start     <= 1'b1;
                        st           <= ST_HDR;
                        hdr_i        <= 16'd1;
                    end
                end

                ST_HDR: begin
                    if (!tx_busy) begin
                        if (hdr_i >= HEADER_LEN) begin
                            rd_addr <= 0;          // payload[0]
                            st      <= ST_RD_WAIT; // cho 1 clk BRAM
                            pay_i   <= 0;
                        end else begin
                            tx_data  <= header_byte(hdr_i);
                            tx_start <= 1'b1;
                            hdr_i    <= hdr_i + 1'b1;
                        end
                    end
                end

                ST_RD_WAIT: begin
                    st <= ST_PAY_SEND;
                end

                ST_PAY_SEND: begin
                    if (!tx_busy) begin
                        tx_data  <= rd_data;
                        tx_start <= 1'b1;

                        if (pay_i == PAYLOAD_LEN-1) begin
                            st <= ST_DONE;
                        end else begin
                            pay_i   <= pay_i + 1'b1;
                            rd_addr <= (pay_i + 1'b1);
                            st      <= ST_RD_WAIT;
                        end
                    end
                end

                ST_DONE: begin
                    if (!tx_busy) begin
                        consume_toggle_uart <= ~consume_toggle_uart;
                        st <= ST_IDLE;
                    end
                end

                default: st <= ST_IDLE;
            endcase
        end
    end

endmodule
