`timescale 1ns/1ps
// Test-only functional model of the SAME-CLOCK/NOREG BRAM configurations in
// edge_detect_96. Not a replacement for vendor simulation or device timing.
// Do not add to the synthesis project or compile with the vendor model.
module PH1P_LOGIC_ERAM #(
    parameter DATA_WIDTH_A=20, DATA_WIDTH_B=DATA_WIDTH_A,
    parameter ADDR_WIDTH_A=10, ADDR_WIDTH_B=ADDR_WIDTH_A,
    parameter DATA_DEPTH_A=2**ADDR_WIDTH_A, DATA_DEPTH_B=DATA_DEPTH_A,
    parameter MODE="DP", REGMODE_A="NOREG", REGMODE_B="NOREG",
    parameter WRITEMODE_A="NORMAL", WRITEMODE_B="NORMAL",
    parameter CLKMODE="SYNC", IMPLEMENT="20K(FAST)"
)(
    input clka, cea, ocea, wea, rsta, bea,
    input clkb, ceb, oceb, web, rstb, beb,
    input [ADDR_WIDTH_A-1:0] addra,
    input [ADDR_WIDTH_B-1:0] addrb,
    input [DATA_WIDTH_A-1:0] dia,
    input [DATA_WIDTH_B-1:0] dib,
    input ecc_sbiterrinj, ecc_dbiterrinj,
    output reg [DATA_WIDTH_A-1:0] doa,
    output reg [DATA_WIDTH_B-1:0] dob,
    output ecc_sbiterr, ecc_dbiterr
);
    reg [DATA_WIDTH_A-1:0] mem [0:DATA_DEPTH_A-1];
    assign ecc_sbiterr=0;
    assign ecc_dbiterr=0;
    initial begin
        if (MODE!="DP" || CLKMODE!="SYNC" || REGMODE_A!="NOREG" ||
            REGMODE_B!="NOREG" || DATA_WIDTH_A!=DATA_WIDTH_B ||
            DATA_DEPTH_A!=DATA_DEPTH_B || ADDR_WIDTH_A!=ADDR_WIDTH_B)
            $fatal(1,"Unsupported test BRAM configuration");
    end
    // Inputs are sampled on the edge, before either port's nonblocking write.
    always @(posedge clka) if (cea) begin
        if (!wea || WRITEMODE_A=="READBEFOREWRITE") doa<=mem[addra];
        else if (WRITEMODE_A=="WRITETHROUGH") doa<=dia;
        if (wea) mem[addra]<=dia;
    end
    always @(posedge clkb) if (ceb) begin
        if (!web || WRITEMODE_B=="READBEFOREWRITE") dob<=mem[addrb];
        else if (WRITEMODE_B=="WRITETHROUGH") dob<=dib;
        if (web) mem[addrb]<=dib;
        if (cea && wea && web && addra==addrb)
            $fatal(1,"Two writes to one BRAM address");
    end
endmodule
