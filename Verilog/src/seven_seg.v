`timescale 1ns / 1ps

module seven_seg(CLK, HEX, display_mode, AN, LCD);
input CLK;
input [31:0] HEX;
input display_mode;
output reg [7:0] AN;
output reg [7:0] LCD;

reg [3:0] four_bit_val;
reg [26:0] blink_rate;
reg [19:0] refresh_rate;
reg led_state;
wire [2:0] led_sel;

always @(posedge CLK)
begin
        refresh_rate <= refresh_rate + 1;
        blink_rate <= blink_rate + 1;
end
assign led_sel = refresh_rate[19:16];

always @(*)
    begin
        case(display_mode)
            1'b1: begin
                case(led_sel) // displaying 8 bytes
                    3'b000: begin
                        AN <= 8'b01111111;
                        four_bit_val <= HEX[31:28];
                    end
                    3'b001: begin
                        AN <= 8'b10111111;
                        four_bit_val <= HEX[27:24];
                    end
                    3'b010: begin
                        AN <= 8'b11011111; 
                        four_bit_val <= HEX[23:20];
                    end
                    3'b011: begin
                        AN   <= 8'b11101111; 
                        four_bit_val <= HEX[19:16];
                    end
                    3'b100: begin
                        AN <= 8'b11110111; 
                        four_bit_val <= HEX[15:12];
                    end
                    3'b101: begin
                        AN <= 8'b11111011; 
                        four_bit_val <= HEX[11:8];
                    end
                    3'b110: begin
                        AN <= 8'b11111101;
                        four_bit_val <= HEX[7:4];
                    end
                    3'b111: begin
                        AN <= 8'b11111110; 
                        four_bit_val <= HEX[3:0];
                   end
                endcase
            end
            2'b10: begin // all zeros
                case(led_sel)
                    3'b000: begin
                        AN <= 8'b01111111;
                        four_bit_val <= 0;
                    end
                    3'b001: begin
                        AN <= 8'b10111111;
                        four_bit_val <= 0;
                    end
                    3'b010: begin
                        AN <= 8'b11011111; 
                        four_bit_val <= 0;
                    end
                    3'b011: begin
                        AN   <= 8'b11101111; 
                        four_bit_val <= 0;
                    end
                    3'b100: begin
                        AN <= 8'b11110111; 
                        four_bit_val <= 0;
                    end
                    3'b101: begin
                        AN <= 8'b11111011; 
                        four_bit_val <= 0;
                    end
                    3'b110: begin
                        AN <= 8'b11111101;
                        four_bit_val <= 0;
                    end
                    3'b111: begin
                        AN <= 8'b11111110; 
                        four_bit_val <= 0;
                   end
                endcase
            end
            1'b0: begin // off
                case(led_sel)
                    3'b000: begin
                        AN <= 8'b11111111;
                    end
                    3'b001: begin
                        AN <= 8'b11111111; 
                    end
                    3'b010: begin
                        AN <= 8'b11111111;
                    end
                    3'b011: begin
                        AN <= 8'b11111111; 
                    end
                    3'b100: begin
                        AN <= 8'b11111111; 
                    end
                    3'b101: begin
                        AN <= 8'b11111111; 
                    end
                    3'b110: begin
                        AN <= 8'b11111111; 
                    end
                    3'b111: begin
                        AN <= 8'b11111111; 
                   end
                endcase
            end 
    endcase
end
    // Cathode patterns of the 7-segment LED display 
always @(*)
begin
    case(four_bit_val)
        4'b0000: LCD <= 8'b10000001; // "0"  
        4'b0001: LCD <= 8'b11001111; // "1" 
        4'b0010: LCD <= 8'b10010010; // "2" 
        4'b0011: LCD <= 8'b10000110; // "3" 
        4'b0100: LCD <= 8'b11001100; // "4" 
        4'b0101: LCD <= 8'b10100100; // "5" 
        4'b0110: LCD <= 8'b10100000; // "6" 
        4'b0111: LCD <= 8'b10001111; // "7" 
        4'b1000: LCD <= 8'b10000000; // "8"     
        4'b1001: LCD <= 8'b10000100; // "9" 
        4'b1010: LCD <= 8'b10001000; // "A"     
        4'b1011: LCD <= 8'b11100000; // "b" 
        4'b1100: LCD <= 8'b10110001; // "C" 
        4'b1101: LCD <= 8'b11000010; // "d" 
        4'b1110: LCD <= 8'b10110000; // "E" 
        4'b1111: LCD <= 8'b10111000; // "F" 
        default: LCD <= 8'b11110111; // "0"
    endcase
end
endmodule


