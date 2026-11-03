`timescale 1ns/1ps
// Aspect-fit nearest-neighbour BMP rasterizer for a contiguous file. A sector
// and a scaled row are cached in synchronous RAM. Output stalls never truncate
// an SD transaction. Bottom-up output matches the SDRAM writer's vertical flip.
module bmp_scale(
    input clk, rst, start, abort,
    input [11:0] src_w, src_h,
    input top_down,
    input [31:0] pixel_offset, base_sector,
    input out_ready,
    output out_valid,
    output [23:0] out_rgb,
    output reg busy, done, error,
    output sd_req,
    output reg [31:0] sd_addr,
    input [7:0] sd_data,
    input sd_valid, sd_end
);
localparam IDLE=0, GEOM=1, GEOM_END=2, ROW=3, DIV_Y=4, ROW_ADDR=5,
           BYTE=6, BYTE_USE=7, RECEIVE=8, STORE=9, STEP_X=10,
           EMIT_READ=11, EMIT=12, ROW_MUL=13, ROW_SUM=14;
reg [3:0] state;
reg [11:0] sw, sh;
reg td, width_limited;
reg [31:0] offset, base;
reg [12:0] stride;
reg [21:0] geom_rem;
reg [9:0] geom_q, dw, dh, left_pad, bottom_pad;
reg [9:0] ox, oy, dx;
reg [21:0] row_rem;
reg [11:0] sy, cached_sy, sx, target_x;
reg row_cached, row_active;
reg [12:0] phase_x;
reg [31:0] cursor;
reg [1:0] component;
reg [23:0] rgb;
reg [7:0] sector_mem [0:511];
reg [23:0] line_mem [0:639];
reg [7:0] sector_q;
reg [23:0] line_q;
reg [22:0] sector_tag;
reg sector_cached, cancel_pending;
reg [9:0] received;
wire [9:0] line_addr = (ox >= left_pad && ox < left_pad + dw) ? ox-left_pad : 10'd0;
wire inside_x = ox >= left_pad && ox < left_pad + dw;
wire [11:0] file_row = td ? sy : sh-12'd1-sy;
reg [11:0] file_row_q;
reg [24:0] row_bytes_q;
wire [21:0] sw480 = ({10'd0,src_w} << 9) - ({10'd0,src_w} << 5);
wire [21:0] sh640 = ({10'd0,src_h} << 9) + ({10'd0,src_h} << 7);
assign sd_req = state == RECEIVE;
assign out_valid = state == EMIT && !abort && !cancel_pending;
assign out_rgb = row_active && inside_x ? line_q : 24'd0;

always @(posedge clk) begin
    if (state == RECEIVE && sd_valid && received < 10'd512)
        sector_mem[received[8:0]] <= sd_data;
    sector_q <= sector_mem[cursor[8:0]];
    if (state == STORE) line_mem[dx] <= rgb;
    line_q <= line_mem[line_addr];
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state<=IDLE; busy<=0; done<=0; error<=0; cancel_pending<=0;
        sw<=0; sh<=0; td<=0; width_limited<=0; offset<=0; base<=0;
        stride<=0; geom_rem<=0; geom_q<=0; dw<=0; dh<=0;
        left_pad<=0; bottom_pad<=0; ox<=0; oy<=0; dx<=0;
        row_rem<=0; sy<=0; cached_sy<=0; sx<=0; target_x<=0;
        row_cached<=0; row_active<=0; phase_x<=0; cursor<=0;
        component<=0; rgb<=0; sector_tag<=0; sector_cached<=0;
        sd_addr<=0; received<=0;
        file_row_q<=0; row_bytes_q<=0;
    end else begin
        done<=0; error<=0;
        // Drain an issued sector before releasing the bus on cancellation.
        if (abort && busy && state == RECEIVE) cancel_pending<=1;
        if (abort && busy && state != RECEIVE) begin
            state<=IDLE; busy<=0; error<=1; cancel_pending<=0;
        end else case (state)
            IDLE: if (start && (abort || src_w==0 || src_h==0 || src_w>2048 || src_h>2048)) begin
                error<=1;
            end else if (start) begin
                sw<=src_w; sh<=src_h; td<=top_down;
                offset<=pixel_offset; base<=base_sector;
                stride<=(({1'b0,src_w} << 1)+{1'b0,src_w}+13'd3) & 13'h1ffc;
                width_limited<=sw480 >= sh640;
                geom_rem<=sw480 >= sh640 ? sh640 : sw480;
                geom_q<=0; row_cached<=0; sector_cached<=0;
                cancel_pending<=0; busy<=1; oy<=0; state<=GEOM;
            end
            // The quotient is at most 640: this small iterative divider
            // replaces a large combinational divider in the setup path.
            GEOM: if (geom_rem >= (width_limited ? sw : sh)) begin
                geom_rem<=geom_rem-(width_limited ? sw : sh);
                geom_q<=geom_q+10'd1;
            end else begin
                dw<=width_limited ? 10'd640 : (geom_q==0 ? 10'd1 : geom_q);
                dh<=width_limited ? (geom_q==0 ? 10'd1 : geom_q) : 10'd480;
                state<=GEOM_END;
            end
            GEOM_END: begin
                left_pad<=(10'd640-dw)>>1;
                bottom_pad<=10'd480-dh-((10'd480-dh)>>1);
                state<=ROW;
            end
            ROW: begin
                ox<=0;
                row_active<=oy>=bottom_pad && oy<bottom_pad+dh;
                if (oy>=bottom_pad && oy<bottom_pad+dh) begin
                    row_rem<=(dh-10'd1-(oy-bottom_pad))*sh;
                    sy<=0; state<=DIV_Y;
                end else state<=EMIT_READ;
            end
            DIV_Y: if (row_rem >= dh) begin
                row_rem<=row_rem-dh; sy<=sy+12'd1;
            end else if (row_cached && sy==cached_sy) state<=EMIT_READ;
            else state<=ROW_ADDR;
            ROW_ADDR: begin
                file_row_q<=file_row; state<=ROW_MUL;
            end
            ROW_MUL: begin
                row_bytes_q<=file_row_q*stride; state<=ROW_SUM;
            end
            ROW_SUM: begin
                cursor<=offset+{7'd0,row_bytes_q}; component<=0;
                sx<=0; target_x<=0; dx<=0; phase_x<=0;
                state<=BYTE;
            end
            BYTE: if (sector_cached && cursor[31:9]==sector_tag)
                state<=BYTE_USE;
            else begin
                sd_addr<=base+{9'd0,cursor[31:9]}; received<=0;
                sector_cached<=0; state<=RECEIVE;
            end
            RECEIVE: begin
                if (sd_valid && received < 10'd513) received<=received+10'd1;
                if (sd_end) begin
                    if (cancel_pending || abort ||
                        (received + (sd_valid ? 10'd1 : 10'd0) != 10'd512)) begin
                        busy<=0; error<=1; state<=IDLE; cancel_pending<=0;
                    end else begin
                        sector_tag<=cursor[31:9]; sector_cached<=1; state<=BYTE;
                    end
                end
            end
            BYTE_USE: begin
                cursor<=cursor+32'd1;
                case (component)
                    0: begin rgb[7:0]<=sector_q; component<=1; state<=BYTE; end
                    1: begin rgb[15:8]<=sector_q; component<=2; state<=BYTE; end
                    2: begin
                        rgb[23:16]<=sector_q; component<=0;
                        if (sx==target_x) state<=STORE;
                        else begin sx<=sx+12'd1; state<=BYTE; end
                    end
                    default: state<=IDLE;
                endcase
            end
            STORE: if (dx==dw-10'd1) begin
                row_cached<=1; cached_sy<=sy; state<=EMIT_READ;
            end else begin
                dx<=dx+10'd1; phase_x<=phase_x+{1'b0,sw}; state<=STEP_X;
            end
            STEP_X: if (phase_x >= dw) begin
                phase_x<=phase_x-dw; target_x<=target_x+12'd1;
            end else if (target_x==sx) state<=STORE;
            else begin sx<=sx+12'd1; state<=BYTE; end
            EMIT_READ: state<=EMIT;
            EMIT: if (out_ready) begin
                if (ox==10'd639) begin
                    if (oy==10'd479) begin busy<=0; done<=1; state<=IDLE; end
                    else begin oy<=oy+10'd1; state<=ROW; end
                end else begin ox<=ox+10'd1; state<=EMIT_READ; end
            end
            default: begin busy<=0; error<=1; state<=IDLE; end
        endcase
    end
end
endmodule
