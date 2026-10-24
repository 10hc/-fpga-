// ============================================================
// sim_eg_logic_bram.v —— 仅用于仿真的 EG_LOGIC_BRAM 行为级替身
//
// 为什么需要它：TD 生成的 IP 文件（font_rom.v、AUDIO_SAMPLE_ROM.v 等）实例化
// 安路原语 EG_LOGIC_BRAM，而该原语的官方仿真模型在 TD 安装目录
//   D:\AnluTD\sim_release\eg4\eg_logic_bram.v
// 是 pragma protect 加密的，ModelSim ASE 载入会报
//   "Instantiation of 'EG_LOGIC_BRAM' failed. The design unit was not found."
// 所以仿真时用本文件顶替（综合时绝对不要加入工程！）。
//
// 用法：vlog sim_eg_logic_bram.v font_rom.v osd_overlay.v tb_xxx.v
//       vsim -c -do "run -all" work.tb_xxx +MIF=<font8x16.mif 绝对路径>
//       （不给 +MIF 时按 INIT_FILE 相对路径找，找不到就全 0，不影响时序/通路验证）
// ============================================================
`timescale 1ns/1ps

module EG_LOGIC_BRAM #(
    parameter DATA_WIDTH_A = 8,
    parameter ADDR_WIDTH_A = 11,
    parameter DATA_DEPTH_A = 2048,
    parameter DATA_WIDTH_B = 8,
    parameter ADDR_WIDTH_B = 11,
    parameter DATA_DEPTH_B = 2048,
    parameter MODE         = "SP",
    parameter REGMODE_A    = "NOREG",
    parameter IMPLEMENT    = "9K",
    parameter DEBUGGABLE   = "NO",
    parameter PACKABLE     = "NO",
    parameter INIT_FILE    = "",
    parameter FILL_ALL     = "NONE"
)(
    input  wire [DATA_WIDTH_A-1:0] dia,
    input  wire [DATA_WIDTH_B-1:0] dib,
    input  wire [ADDR_WIDTH_A-1:0] addra,
    input  wire [ADDR_WIDTH_B-1:0] addrb,
    input  wire                    cea,
    input  wire                    ceb,
    input  wire                    ocea,
    input  wire                    oceb,
    input  wire                    clka,
    input  wire                    clkb,
    input  wire                    wea,
    input  wire                    web,
    input  wire                    bea,
    input  wire                    beb,
    input  wire                    rsta,
    input  wire                    rstb,
    output reg  [DATA_WIDTH_A-1:0] doa,
    output reg  [DATA_WIDTH_B-1:0] dob
);
    reg [DATA_WIDTH_A-1:0] mem [0:(1<<ADDR_WIDTH_A)-1];
    reg [1023:0] mif_path;
    integer i;

    initial begin
        for (i = 0; i < (1<<ADDR_WIDTH_A); i = i + 1)
            mem[i] = {DATA_WIDTH_A{1'b0}};
        if (!$value$plusargs("MIF=%s", mif_path)) begin
            if (INIT_FILE != "")
                mif_path = INIT_FILE;
            else
                mif_path = 0;
        end
        if (mif_path != 0) begin
            $display("[sim_EG_LOGIC_BRAM] 载入初始化文件 %0s", mif_path);
            $readmemh(mif_path, mem);
        end
    end

    // A 口：同步读，写优先（REG MODE 不影响此处功能验证）
    always @(posedge clka) begin
        if (cea) begin
            if (wea) begin
                mem[addra] <= dia;
                doa        <= dia;
            end else begin
                doa <= mem[addra];
            end
        end
        if (rsta) doa <= {DATA_WIDTH_A{1'b0}};
    end

    // B 口：本项目未用
    always @(posedge clkb) begin
        if (ceb) begin
            if (web) mem[addrb] <= dib;
            dob <= mem[addrb];
        end
        if (rstb) dob <= {DATA_WIDTH_B{1'b0}};
    end
endmodule