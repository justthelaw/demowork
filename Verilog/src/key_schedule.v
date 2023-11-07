`timescale 1ns / 1ps

module key_schedule(
    input [11:0] round_num,
    input [127:0] key,
    output [31:0] g
    );
    reg [31:0] g;
    wire [31:0] s_byte;
    reg [31:0] RC;
    
    // using s_byte modules to find values for key scheduling
    sub_byte S_03(.SUB_ADDR(key[31:24]), .SUB_BYTE(s_byte[31:24]));
    sub_byte S_02(.SUB_ADDR(key[23:16]), .SUB_BYTE(s_byte[23:16]));
    sub_byte S_01(.SUB_ADDR(key[15:8]),  .SUB_BYTE(s_byte[15:8]));
    sub_byte S_LSB(.SUB_ADDR(key[7:0]),  .SUB_BYTE(s_byte[7:0]));
    
    always @(key) begin
        case(round_num) // assigning value to be XORd
            1:
                RC <= 8'b00000001;
            2:
                RC <= 8'b00000010;
            3:
                RC <= 8'b00000100;
            4:
                RC <= 8'b00001000;
            5:
                RC <= 8'b00010000;
            6:
                RC <= 8'b00100000;
            7:
                RC <= 8'b01000000;
            8:
                RC <= 8'b10000000;
            9:
                RC <= 8'b00011011;
            10:
                RC <= 8'b00110110;
        endcase
        // finding value of g to be sent back
        g[31:24] <= s_byte[23:16] ^ RC;
        g[23:16] <= s_byte[15:8];
        g[15:8] <= s_byte[7:0];
        g[7:0] <= s_byte[31:24];
    end
   
endmodule
