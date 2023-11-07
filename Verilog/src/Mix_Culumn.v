module Mix_Culumn(in, out);
	
	input [127:0]in;
	output [127:0]out;
	
	
	
	function [7:0]Mult_GF;
		input [3:0]B;
		input [7:0]A;  
		reg [8:0]Mid;
		begin
			case(B)
				1:
				begin
					Mid = {1'b0,A};
				end
				2:
				begin
					Mid = {A,1'b0};
				end
				3:
				begin
					Mid = {A,1'b0} ^ {1'b0 , A};
				end
			endcase
			
			if(Mid[8] == 1'b1)
				Mult_GF = Mid[7:0] ^ 8'b0001_1011;
			else
				Mult_GF = Mid[7:0];
		end
	endfunction
  
  
	assign out[127:120] = Mult_GF(2, in[127:120]) ^ Mult_GF(3, in[119:112]) ^ Mult_GF(1, in[111:104]) ^ Mult_GF(1, in[103:96]); 
	assign out[119:112] = Mult_GF(1, in[127:120]) ^ Mult_GF(2, in[119:112]) ^ Mult_GF(3, in[111:104]) ^ Mult_GF(1, in[103:96]);
	assign out[111:104] = Mult_GF(1, in[127:120]) ^ Mult_GF(1, in[119:112]) ^ Mult_GF(2, in[111:104]) ^ Mult_GF(3, in[103:96]);
	assign out[103:96]  = Mult_GF(3, in[127:120]) ^ Mult_GF(1, in[119:112]) ^ Mult_GF(1, in[111:104]) ^ Mult_GF(2, in[103:96]);
	
	assign out[95:88]  = Mult_GF(2, in[95:88]) ^ Mult_GF(3, in[87:80]) ^ Mult_GF(1, in[79:72]) ^ Mult_GF(1, in[71:64]); 
	assign out[87:80]  = Mult_GF(1, in[95:88]) ^ Mult_GF(2, in[87:80]) ^ Mult_GF(3, in[79:72]) ^ Mult_GF(1, in[71:64]);
	assign out[79:72]  = Mult_GF(1, in[95:88]) ^ Mult_GF(1, in[87:80]) ^ Mult_GF(2, in[79:72]) ^ Mult_GF(3, in[71:64]);
	assign out[71:64]  = Mult_GF(3, in[95:88]) ^ Mult_GF(1, in[87:80]) ^ Mult_GF(1, in[79:72]) ^ Mult_GF(2, in[71:64]);
	
	assign out[63:56]  = Mult_GF(2, in[63:56]) ^ Mult_GF(3, in[55:48]) ^ Mult_GF(1, in[47:40]) ^ Mult_GF(1, in[39:32]); 
	assign out[55:48]  = Mult_GF(1, in[63:56]) ^ Mult_GF(2, in[55:48]) ^ Mult_GF(3, in[47:40]) ^ Mult_GF(1, in[39:32]);
	assign out[47:40]  = Mult_GF(1, in[63:56]) ^ Mult_GF(1, in[55:48]) ^ Mult_GF(2, in[47:40]) ^ Mult_GF(3, in[39:32]);
	assign out[39:32]  = Mult_GF(3, in[63:56]) ^ Mult_GF(1, in[55:48]) ^ Mult_GF(1, in[47:40]) ^ Mult_GF(2, in[39:32]);
		
	assign out[31:24]  = Mult_GF(2, in[31:24]) ^ Mult_GF(3, in[23:16]) ^ Mult_GF(1, in[15:8]) ^ Mult_GF(1, in[7:0]); 
	assign out[23:16]  = Mult_GF(1, in[31:24]) ^ Mult_GF(2, in[23:16]) ^ Mult_GF(3, in[15:8]) ^ Mult_GF(1, in[7:0]);
	assign out[15:8]   = Mult_GF(1, in[31:24]) ^ Mult_GF(1, in[23:16]) ^ Mult_GF(2, in[15:8]) ^ Mult_GF(3, in[7:0]);
	assign out[7:0]    = Mult_GF(3, in[31:24]) ^ Mult_GF(1, in[23:16]) ^ Mult_GF(1, in[15:8]) ^ Mult_GF(2, in[7:0]);	
	
endmodule
