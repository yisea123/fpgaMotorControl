module pwm (input wire clk, input wire [7:0] in,
			    output reg out);
	wire [10:0] local_input;
   reg [10:0] clk_count;
	assign local_input = in << 3;//11'b00010000000; //this is 2.56 microseconds
	
	always @(posedge clk)
	   clk_count <= clk_count+1'b1;
		
	always @(posedge clk)
		if(clk_count<local_input) out<=0;
		else             out<=1;

endmodule 