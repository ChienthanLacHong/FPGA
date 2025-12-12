`timescale 1ns / 1ps

module vga_interface(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sobel,

    // asyn_fifo IO
    input  wire        empty_fifo,
    input  wire [15:0] din,
    output wire        clk_vga,
    output reg         rd_en,
    input  wire [7:0]  threshold,

    // VGA output
    output reg  [4:0]  vga_out_r,
    output reg  [5:0]  vga_out_g,
    output reg  [4:0]  vga_out_b,
    output wire        vga_out_vs,
    output wire        vga_out_hs,

    // stream for BRAM capture
    output wire        sobel_stream_valid_vga,
    output wire [7:0]  sobel_stream_pixel_vga, // LSB=edge_bit
    output wire        sobel_frame_start_vga
);

    // FSM state declarations
    localparam delay   = 2'd0,
               idle    = 2'd1,
               display = 2'd2;

    reg  [1:0]  state_q, state_d;
    wire [11:0] pixel_x, pixel_y;
    wire        video_on;

    // clock out from DCM
    wire clk_out;

    // sobel magnitude 8-bit
    wire [7:0] sobel_mag = din[7:0];
    wire       edge_bit  = (sobel_mag > threshold);

    // register operations
    always @(posedge clk_out or negedge rst_n) begin
        if(!rst_n) state_q <= delay;
        else       state_q <= state_d;
    end

    // FSM next-state logic
    always @* begin
        state_d   = state_q;
        rd_en     = 1'b0;

        vga_out_r = 5'd0;
        vga_out_g = 6'd0;
        vga_out_b = 5'd0;

        case(state_q)
            delay: begin
                if(pixel_x == 12'd1 && pixel_y == 12'd1)
                    state_d = idle;
            end

            idle: begin
                if(pixel_x == 12'd1 && pixel_y == 12'd0 && !empty_fifo) begin
                    if(sobel) begin
                        vga_out_r = edge_bit ? 5'h1F : 5'd0;
                        vga_out_g = edge_bit ? 6'h3F : 6'd0;
                        vga_out_b = edge_bit ? 5'h1F : 5'd0;
                    end else begin
                        vga_out_r = din[15:11];
                        vga_out_g = din[10:5];
                        vga_out_b = din[4:0];
                    end
                    rd_en   = 1'b1;
                    state_d = display;
                end
            end

            display: begin
                if(pixel_x >= 12'd1 && pixel_x <= 12'd640 && pixel_y < 12'd480) begin
                    if(sobel) begin
                        vga_out_r = edge_bit ? 5'h1F : 5'd0;
                        vga_out_g = edge_bit ? 6'h3F : 6'd0;
                        vga_out_b = edge_bit ? 5'h1F : 5'd0;
                    end else begin
                        vga_out_r = din[15:11];
                        vga_out_g = din[10:5];
                        vga_out_b = din[4:0];
                    end
                    rd_en = 1'b1;
                end
            end

            default: state_d = delay;
        endcase
    end

    assign clk_vga = clk_out;

    // frame start pulse: first visible pixel in frame
    assign sobel_frame_start_vga = (pixel_x == 12'd1) && (pixel_y == 12'd0);

    // valid pixel for capture: sobel mode + inside visible area + fifo has data
    assign sobel_stream_valid_vga =
        sobel &&
        (pixel_x >= 12'd1) && (pixel_x <= 12'd640) &&
        (pixel_y <  12'd480) &&
        !empty_fifo;

    // output only 1 bit edge in LSB
    assign sobel_stream_pixel_vga = {7'd0, edge_bit};

    // module instantiations
    vga_core m0 (
        .clk     (clk_out),
        .rst_n   (rst_n),
        .hsync   (vga_out_hs),
        .vsync   (vga_out_vs),
        .video_on(video_on),
        .pixel_x (pixel_x),
        .pixel_y (pixel_y)
    );

    dcm_25MHz m1 (
        .clk     (clk),
        .clk_out (clk_out),
        .RESET   (1'b0),     // hoac noi RESET/LOCKED neu ban co
        .LOCKED  ()
    );

endmodule
