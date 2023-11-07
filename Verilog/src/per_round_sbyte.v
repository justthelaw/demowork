`timescale 1ns / 1ps


module per_round_sbyte(
    input [127:0] full_hex,
    output [127:0] return_hex
    );
    
    // takes all 16 pairs of 2 bytes and finds the s_byte for each of them
    sub_byte S_MSB(.SUB_ADDR(full_hex[127:120]), .SUB_BYTE(return_hex[127:120]));
    sub_byte S_14(.SUB_ADDR(full_hex[119:112]), .SUB_BYTE(return_hex[119:112]));
    sub_byte S_13(.SUB_ADDR(full_hex[111:104]), .SUB_BYTE(return_hex[111:104]));
    sub_byte S_12(.SUB_ADDR(full_hex[103:96]), .SUB_BYTE(return_hex[103:96]));
    sub_byte S_11(.SUB_ADDR(full_hex[95:88]), .SUB_BYTE(return_hex[95:88]));
    sub_byte S_10(.SUB_ADDR(full_hex[87:80]), .SUB_BYTE(return_hex[87:80]));
    sub_byte S_09(.SUB_ADDR(full_hex[79:72]), .SUB_BYTE(return_hex[79:72]));
    sub_byte S_08(.SUB_ADDR(full_hex[71:64]), .SUB_BYTE(return_hex[71:64]));
    sub_byte S_07(.SUB_ADDR(full_hex[63:56]), .SUB_BYTE(return_hex[63:56]));
    sub_byte S_06(.SUB_ADDR(full_hex[55:48]), .SUB_BYTE(return_hex[55:48]));
    sub_byte S_05(.SUB_ADDR(full_hex[47:40]), .SUB_BYTE(return_hex[47:40]));
    sub_byte S_04(.SUB_ADDR(full_hex[39:32]), .SUB_BYTE(return_hex[39:32]));
    sub_byte S_03(.SUB_ADDR(full_hex[31:24]), .SUB_BYTE(return_hex[31:24]));
    sub_byte S_02(.SUB_ADDR(full_hex[23:16]), .SUB_BYTE(return_hex[23:16]));
    sub_byte S_01(.SUB_ADDR(full_hex[15:8]),  .SUB_BYTE(return_hex[15:8]));
    sub_byte S_LSB(.SUB_ADDR(full_hex[7:0]),  .SUB_BYTE(return_hex[7:0]));
    
    
endmodule
