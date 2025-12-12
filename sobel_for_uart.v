`timescale 1ns/1ps

module sobel_frame_capture_bram_1bpp_lock_dualclk #(
    parameter integer WIDTH       = 640,
    parameter integer HEIGHT      = 480,
    parameter integer PAYLOAD_LEN = (WIDTH*HEIGHT)/8,  // 38400
    parameter integer ADDR_WIDTH  = 16
)(
    // write domain (VGA)
    input  wire                  clk_vga,
    input  wire                  rst_n,
    input  wire                  sobel_stream_valid,
    input  wire [7:0]            sobel_stream_pixel,   // LSB=edge_bit (0/1)
    input  wire                  sobel_frame_start,

    // consume from UART domain (toggle once per finished frame send)
    input  wire                  consume_toggle_uart,

    output reg                   frame_ready_vga,   // FULL
    output reg                   locked_vga,        // FULL (alias)
    output reg                   bram_empty_vga,    // EMPTY (new)
    output reg  [31:0]           frame_id_vga,

    // read domain (UART)
    input  wire                  clk_uart,
    input  wire [ADDR_WIDTH-1:0] rd_addr,
    output reg  [7:0]            rd_data
);

    localparam integer LAST_ADDR = PAYLOAD_LEN - 1;
    localparam integer DEPTH     = PAYLOAD_LEN;

    // BRAM
    (* ram_style = "block" *)
    reg [7:0] ram [0:DEPTH-1];

    // write pack
    reg [ADDR_WIDTH-1:0] wr_addr;
    reg [7:0]            pack_reg;
    reg [2:0]            pack_cnt;
    reg                  capturing;

    wire edge_bit = sobel_stream_pixel[0];

    // -----------------------------
    // CDC: sync consume_toggle_uart into clk_vga and detect toggle edge correctly
    // -----------------------------
    reg [2:0] cons_sync;
    reg       cons_prev;          // previous synced level
    wire      cons_level = cons_sync[2];
    wire      consume_edge_vga = cons_level ^ cons_prev;

    always @(posedge clk_vga or negedge rst_n) begin
        if (!rst_n) begin
            cons_sync <= 3'b000;
            cons_prev <= 1'b0;
        end else begin
            cons_sync <= {cons_sync[1:0], consume_toggle_uart};
            cons_prev <= cons_level;   // delay 1 cycle for edge detect
        end
    end

    // -----------------------------
    // Write FSM: EMPTY -> capture full frame -> FULL (locked) -> wait consume -> EMPTY
    // -----------------------------
    always @(posedge clk_vga or negedge rst_n) begin
        if (!rst_n) begin
            frame_ready_vga <= 1'b0;
            locked_vga      <= 1'b0;
            bram_empty_vga  <= 1'b1;

            frame_id_vga    <= 32'd0;

            wr_addr         <= {ADDR_WIDTH{1'b0}};
            pack_reg        <= 8'd0;
            pack_cnt        <= 3'd0;
            capturing       <= 1'b0;
        end else begin
            // When UART finished sending a full frame, it toggles consume_toggle_uart
            // -> mark BRAM EMPTY again
            if (consume_edge_vga) begin
                frame_ready_vga <= 1'b0;
                locked_vga      <= 1'b0;
                bram_empty_vga  <= 1'b1;

                capturing       <= 1'b0;
                wr_addr         <= 0;
                pack_reg        <= 0;
                pack_cnt        <= 0;
            end

            // Start capture ONLY when EMPTY
            // If a new frame_start comes while capturing, we restart capture from 0
            // to avoid mixing frames (drop partial).
            if (sobel_frame_start) begin
                if (bram_empty_vga) begin
                    capturing <= 1'b1;
                    wr_addr   <= 0;
                    pack_reg  <= 0;
                    pack_cnt  <= 0;
                end
                // if FULL, ignore frame_start completely
            end

            // Capture pixels only if capturing and still EMPTY
            if (capturing && bram_empty_vga && sobel_stream_valid) begin
                if (pack_cnt == 3'd7) begin
                    // write packed byte (MSB-first)
                    if (wr_addr < DEPTH) begin
                        ram[wr_addr] <= {pack_reg[6:0], edge_bit};
                    end

                    pack_cnt <= 0;
                    pack_reg <= 0;

                    if (wr_addr == LAST_ADDR[ADDR_WIDTH-1:0]) begin
                        // FULL
                        capturing       <= 1'b0;
                        frame_ready_vga <= 1'b1;
                        locked_vga      <= 1'b1;
                        bram_empty_vga  <= 1'b0;
                        frame_id_vga    <= frame_id_vga + 1'b1;
                    end else begin
                        wr_addr <= wr_addr + 1'b1;
                    end
                end else begin
                    pack_reg <= {pack_reg[6:0], edge_bit};
                    pack_cnt <= pack_cnt + 1'b1;
                end
            end

            // If FULL, force stop capturing
            if (locked_vga) begin
                capturing <= 1'b0;
            end
        end
    end

    // -----------------------------
    // Read port (1-cycle sync read)
    // -----------------------------
    always @(posedge clk_uart) begin
        if (rd_addr < DEPTH) rd_data <= ram[rd_addr];
        else                 rd_data <= 8'h00;
    end

endmodule
