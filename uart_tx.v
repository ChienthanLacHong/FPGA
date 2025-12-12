`timescale 1ns/1ps

module uart_tx #(
    parameter integer CLK_FREQ_HZ = 50000000,
    parameter integer BAUD        = 2000000
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_start,
    input  wire [7:0] data_in,
    output reg        tx_busy,
    output reg        tx_done,
    output reg        tx_line
);
    localparam integer DIV = (CLK_FREQ_HZ / BAUD); // 50e6/2e6 = 25

    reg [31:0] div_cnt;
    reg [3:0]  bit_pos;      // 0=start, 1..8=data, 9=stop
    reg [9:0]  shreg;        // [0]=start, [8:1]=data, [9]=stop

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 0;
            bit_pos <= 0;
            shreg   <= 10'h3FF;
            tx_line <= 1'b1;
            tx_busy <= 1'b0;
            tx_done <= 1'b0;
        end else begin
            tx_done <= 1'b0;

            if (!tx_busy) begin
                if (tx_start) begin
                    // start(0) + data LSB-first + stop(1)
                    shreg   <= {1'b1, data_in, 1'b0};
                    tx_line <= 1'b0;      // start bit ngay lap tuc
                    tx_busy <= 1'b1;
                    bit_pos <= 0;
                    div_cnt <= 0;
                end
            end else begin
                if (div_cnt == DIV-1) begin
                    div_cnt <= 0;

                    if (bit_pos == 4'd9) begin
                        // het 1 chu ky stop bit -> ket thuc frame
                        tx_busy <= 1'b0;
                        tx_done <= 1'b1;
                        tx_line <= 1'b1;
                        bit_pos <= 0;
                    end else begin
                        // chuyen sang bit tiep theo
                        bit_pos <= bit_pos + 1'b1;

                        // bit tiep theo nam o shreg[1] (vi shreg[0] dang la bit hien tai)
                        tx_line <= shreg[1];

                        // shift phai, chen '1' de giu idle/stop
                        shreg   <= {1'b1, shreg[9:1]};
                    end
                end else begin
                    div_cnt <= div_cnt + 1'b1;
                end
            end
        end
    end
endmodule
