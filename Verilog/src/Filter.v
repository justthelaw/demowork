// ==========================================================================
// File Name: small_filter.v
// MODULE: small_filter_without_mem 
// Description:                                                              
//       This module does not allow noise which pass from I/O 
// ==========================================================================
 module Filter(	
      // Inputs
      clk,data_in,
      // Outputs
      data_out,data_edge
);

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
 input clk,data_in;
//Outputs
 output  data_out, data_edge;
 
/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/
 parameter   wd=3;
 parameter   n=7;
 parameter   bound=5;
 
/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires

// Internal Registers
 reg holder;
 reg[wd-1:0] counter;
 

 
 
/*****************************************************************************
 *                         sequential logic                                  *
 *****************************************************************************/ 
	always @(posedge clk)
		begin
			if(data_in == 1'b1 && counter < n)
				counter <= counter + {{(wd-1){1'd0}},1'd1};
			else if(data_in == 1'b0 && counter > 0)	
				counter <= counter + {(wd){1'd1}};
			holder <= data_out;				
		end
 /*****************************************************************************
 *                         combinational logic                                *
 *****************************************************************************/ 
    assign data_out		= (counter <= n-bound)? 1'b0:
						  (counter >= bound)? 1'b1 : holder;
    assign data_edge 	= holder ^ data_out;
endmodule
    