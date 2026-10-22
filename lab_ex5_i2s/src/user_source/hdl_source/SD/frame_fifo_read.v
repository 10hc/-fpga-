`timescale 1ns/1ps
module frame_fifo_read
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter ADDR_BITS              = 21,
	parameter BURST_BITS             = 9,
	parameter FIFO_DEPTH             = 512,
	parameter BURST_SIZE             = 256,
	parameter FRAME_WIDTH            = 640   // 转场用：每行像素数
)               
(
	input                            rst,                  
	input                            mem_clk,                    // external memory controller user interface clock
	input							 Sdr_init_done,
	input							 Sdr_init_ref_vld,
    input							 Sdr_busy,
    input							 Sdr_rd_en,
	input							 App_wr_busy,
    output							 O_rd_busy,

	output 							 App_rd_en,
	output  [ADDR_BITS - 1:0]		 App_rd_addr,
	
	input                            read_req,                   // data read module read request,keep '1' until read_req_ack = '1'
	output reg                       read_req_ack,               // data read module read request response
	output                           read_finish,                // data read module read request finish
	input[ADDR_BITS - 1:0]           read_addr_0,                // data read module read request base address 0, used when read_addr_index = 0
	input[ADDR_BITS - 1:0]           read_addr_1,                // data read module read request base address 1, used when read_addr_index = 1
	input[ADDR_BITS - 1:0]           read_addr_2,                // data read module read request base address 1, used when read_addr_index = 2
	input[ADDR_BITS - 1:0]           read_addr_3,                // data read module read request base address 1, used when read_addr_index = 3
	input[1:0]                       read_addr_index,            // select valid base address from read_addr_0 read_addr_1 read_addr_2 read_addr_3
	input[ADDR_BITS - 1:0]           read_len,                   // data read module read request data length
	output reg                       fifo_aclr,                  // to fifo asynchronous clear
	input[9:0]                       wrusedw,                    // from fifo write used words

	// ===== 转场（新增）=====
	input                            trans_en,                  // 转场使能
	input[1:0]                       trans_effect,              // 0=滑动 1=栅格 2=淡出阶段(整行旧图)
	input[9:0]                       trans_offset,              // 滑动=像素偏移；栅格=对角线进度
	input[ADDR_BITS - 1:0]           trans_old_base,            // 旧缓冲基址
	input[ADDR_BITS - 1:0]           trans_new_base,             // 新缓冲基址
	output[3:0]   dbg_state,
	output        dbg_wrbusy,
	output        dbg_req2,
	output        dbg_init
);
localparam ONE                       = 256'd1;
localparam ZERO                      = 256'd0;
localparam [9:0] FW                  = FRAME_WIDTH;             // 640，10 位便于运算
//read state machine code
localparam S_IDLE                    = 0;
localparam S_ACK                     = 1;
localparam S_CHECK_FIFO              = 2;
localparam S_READ_BURST              = 3;
localparam S_READ_BURST_END          = 4;
localparam S_END                     = 5;

reg                                  read_req_d0;
reg                                  read_req_d1;
reg                                  read_req_d2;
reg[ADDR_BITS - 1:0]                 read_len_d0;
reg[ADDR_BITS - 1:0]                 read_len_d1;
reg[ADDR_BITS - 1:0]                 read_len_latch;
reg[ADDR_BITS - 1:0]                 read_cnt;
reg[3:0]                             state;
reg[1:0]                             read_addr_index_d0;
reg[1:0]                             read_addr_index_d1;
reg [ADDR_BITS - 1:0]	 App_rd_addr_r;

// ===== 转场：行内字计数 / 绝对行基址 / 行号 + 帧首锁存的转场参数 =====
reg [9:0]                            line_word;                  // 0..639 当前行内第几个字
reg [8:0]                            line_cnt;                   // 当前行号 0..479
reg [ADDR_BITS - 1:0]                old_line_base;              // 旧图当前行绝对地址
reg [ADDR_BITS - 1:0]                new_line_base;              // 新图当前行绝对地址
reg [9:0]                            slide_split;                // 滑动转场旧图区宽度
reg                                  trans_en_l;                 // 帧首锁存（整帧恒定）
reg [1:0]                            trans_effect_l;
reg [9:0]                            trans_offset_l;
reg [5:0]                            grid_col_block;             // 0..39
reg [4:0]                            grid_row_block;             // 0..29
reg                                  grid_use_new;

reg [BURST_BITS - 1:0]				burst_cnt;
wire								rd_burst_finish;
reg App_rd_en_r;
reg App_rd_en_d0;


wire rd_vld;
reg [3:0] rd_delay;

assign App_rd_addr = {App_rd_addr_r[ADDR_BITS - 1:0]};
assign rd_vld = (state == S_READ_BURST && burst_cnt >= BURST_SIZE);

assign O_rd_busy = (state == S_READ_BURST);
assign rd_burst_finish = (rd_vld && rd_delay == 4'd10);
assign read_finish = (state == S_END) ? 1'b1 : 1'b0;
assign App_rd_en = App_rd_en_d0;

always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		read_req_d0    <=  1'b0;
		read_req_d1    <=  1'b0;
		read_req_d2    <=  1'b0;
		read_len_d0    <=  ZERO[ADDR_BITS - 1:0];
		read_len_d1    <=  ZERO[ADDR_BITS - 1:0];
		read_addr_index_d0 <= 2'b00;
		read_addr_index_d1 <= 2'b00;
	end
	else
	begin
		read_req_d0    <=  read_req;
		read_req_d1    <=  read_req_d0;
		read_req_d2    <=  read_req_d1;     
		read_len_d0    <=  read_len;
		read_len_d1    <=  read_len_d0; 
		read_addr_index_d0 <= read_addr_index;
		read_addr_index_d1 <= read_addr_index_d0;
	end 
end

always @(posedge mem_clk or posedge rst)
begin
	if(rst || App_rd_en)begin
        rd_delay <= 4'd0;
    end
    else if(rd_delay < 4'd10)begin
    	rd_delay <= rd_delay + 1'b1;
    end
end

always @(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		burst_cnt <= ZERO[BURST_BITS - 1:0];
		App_rd_addr_r <= ZERO[ADDR_BITS - 1:0];
		App_rd_en_d0 <= 1'b0;
		line_word <= 10'd0;
		line_cnt  <= 9'd0;
		old_line_base <= ZERO[ADDR_BITS - 1:0];
		new_line_base <= ZERO[ADDR_BITS - 1:0];
		slide_split   <= FW;
		grid_col_block <= 6'd0;
		grid_row_block <= 5'd0;
		grid_use_new   <= 1'b0;
		trans_en_l     <= 1'b0;
		trans_effect_l <= 2'd0;
		trans_offset_l <= 10'd0;
	end
	else begin
	
		if(state == S_CHECK_FIFO)
			burst_cnt <= ZERO[BURST_BITS - 1:0];
		else if(App_rd_en)
			burst_cnt <= burst_cnt + 1'b1;
		else
			burst_cnt <= burst_cnt;
		//
		if(state == S_ACK)
			begin
				if(read_addr_index_d1 == 2'd0)
					App_rd_addr_r <= read_addr_0;
				else if(read_addr_index_d1 == 2'd1)
					App_rd_addr_r <= read_addr_1;
				else if(read_addr_index_d1 == 2'd2)
					App_rd_addr_r <= read_addr_2;
				else if(read_addr_index_d1 == 2'd3)
					App_rd_addr_r <= read_addr_3;
				// ===== 转场：帧首锁存参数（整帧恒定，避免帧中跳变）=====
				trans_en_l     <= trans_en;
				trans_effect_l <= trans_effect;
				trans_offset_l <= trans_offset;
				if (trans_en) begin
					// 首字地址：第0行第0字
					// 栅格：进度0时 0 号块未揭示 → 旧；滑动/淡出：旧+偏移
					if (trans_effect == 2'd1)
						App_rd_addr_r <= (trans_offset == 10'd0) ? trans_old_base : trans_new_base;
					else if (trans_effect == 2'd0 && trans_offset >= FW)
						App_rd_addr_r <= trans_new_base;
					else
						App_rd_addr_r <= trans_old_base + trans_offset;
					old_line_base <= (trans_effect == 2'd1) ? trans_old_base
					                                        : trans_old_base + trans_offset;
					new_line_base <= trans_new_base;
					slide_split   <= FW - trans_offset;
					grid_col_block <= 6'd0;
					grid_row_block <= 5'd0;
					grid_use_new   <= (trans_offset != 10'd0);
				end
				line_word <= 10'd0;
				line_cnt  <= 9'd0;
			end
		else if(App_rd_en)
			begin
				if (trans_en_l) begin
					if (trans_effect_l == 2'd1) begin
						// 栅格：同一个 16×16 块内地址只需 +1，仅在块边界选源。
						if (line_word == (FW - 1)) begin
							App_rd_addr_r <= ({1'b0, grid_row_block} +
							                  ((line_cnt[3:0] == 4'hF) ? 1'b1 : 1'b0)
							                  < trans_offset_l[6:0])
							                 ? new_line_base + FW
							                 : old_line_base + FW;
						end else if (line_word[3:0] == 4'hF) begin
							if (({1'b0, grid_row_block} + {1'b0, grid_col_block} + 1'b1) < trans_offset_l[6:0])
								App_rd_addr_r <= new_line_base + line_word + 1'b1;
							else
								App_rd_addr_r <= old_line_base + line_word + 1'b1;
						end else begin
							App_rd_addr_r <= App_rd_addr_r + 1'b1;
						end
					end else begin
						// 滑动/淡出：行内绝大多数像素只需地址 +1；
						// 仅在“旧图段切到新图段”和行尾两个边界重算绝对地址。
						if (line_word == (FW - 1)) begin
							App_rd_addr_r <= (trans_effect_l == 2'd0 && slide_split == 10'd0)
							                 ? new_line_base + FW
							                 : old_line_base + FW;
						end else if (trans_effect_l == 2'd0 &&
						             slide_split != 10'd0 &&
						             slide_split != FW &&
						             line_word == slide_split - 10'd1) begin
							App_rd_addr_r <= new_line_base;
						end else begin
							App_rd_addr_r <= App_rd_addr_r + 1'b1;
						end
					end

					// ---- 更新行列计数 ----
					if (line_word == (FW - 1)) begin
						line_word <= 10'd0;
						line_cnt  <= line_cnt + 1'b1;
						old_line_base <= old_line_base + FW;
						new_line_base <= new_line_base + FW;
						if (trans_effect_l == 2'd1) begin
							grid_col_block <= 6'd0;
							if (line_cnt[3:0] == 4'hF)
								grid_row_block <= grid_row_block + 1'b1;
							grid_use_new <= ({1'b0, grid_row_block} +
							                 ((line_cnt[3:0] == 4'hF) ? 1'b1 : 1'b0)
							                 < trans_offset_l[6:0]);
						end
					end else if (trans_effect_l == 2'd1 && line_word[3:0] == 4'hF) begin
						line_word <= line_word + 1'b1;
						grid_col_block <= grid_col_block + 1'b1;
						grid_use_new <= ({1'b0, grid_row_block} + {1'b0, grid_col_block} + 1'b1
						                 < trans_offset_l[6:0]);
					end else begin
						line_word <= line_word + 1'b1;
					end
				end else begin
					App_rd_addr_r <= App_rd_addr_r + 1'b1;
				end
			end
		else
			App_rd_addr_r <= App_rd_addr_r;
		//
		if(App_rd_en_r && burst_cnt + App_rd_en < BURST_SIZE)
			App_rd_en_d0 <= 1'b1;
		else
			App_rd_en_d0 <= 1'b0;
	
	end		
end

always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		read_len_latch <= ZERO[ADDR_BITS - 1:0];
		App_rd_en_r <= 1'b0;
		read_cnt <= ZERO[ADDR_BITS - 1:0];
		fifo_aclr <= 1'b0;
		read_req_ack <= 1'b0;
	end
	else
		case(state)
			S_IDLE:
			begin
				if(read_req_d2 == 1'b1 && Sdr_init_done)
				begin
					state <= S_ACK;
				end
				read_req_ack <= 1'b0;
			end
			S_ACK:
			begin
				if(read_req_d2 == 1'b0)
				begin
					state <= S_CHECK_FIFO;
					fifo_aclr <= 1'b0;
					read_req_ack <= 1'b0;
				end
				else
				begin
					read_req_ack <= 1'b1;
					fifo_aclr <= 1'b1;
					read_len_latch <= read_len_d1;
				end
				read_cnt <= ZERO[ADDR_BITS - 1:0];
			end
			S_CHECK_FIFO:
			begin
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				else if(wrusedw < (FIFO_DEPTH - BURST_SIZE) && ~App_wr_busy)
				begin
					state <= S_READ_BURST;
					App_rd_en_r <= 1'b1;
				end
			end
			S_READ_BURST:
			begin
				if(rd_burst_finish == 1'b1)
				begin
					App_rd_en_r <= 1'b0;
					state <= S_READ_BURST_END;
					read_cnt <= read_cnt + BURST_SIZE[ADDR_BITS - 1:0];
				end     
			end
			S_READ_BURST_END:
			begin
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				else if(read_cnt < read_len_latch)
				begin
					state <= S_CHECK_FIFO;
				end
				else
				begin
					state <= S_END;
				end
			end
			S_END:
			begin
				state <= S_IDLE;
			end
			default:
				state <= S_IDLE;
		endcase
end
	assign dbg_state  = state;
	assign dbg_wrbusy = App_wr_busy;
	assign dbg_req2   = read_req_d2;
	assign dbg_init   = Sdr_init_done;
endmodule
