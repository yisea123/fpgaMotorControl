module pwm (
input wire clk, 
input wire [7:0] in,
//output out,
output reg out = 0,
input wire rst);

	wire [10:0] local_input;
   reg [10:0] clk_count;
	assign local_input = in << 3;//11'b00010000000; //this is 2.56 microseconds
	
	//daniel edits
	//reg out_reg = 0;
	//assign out = out_reg;
	
	always @(posedge clk)
	   clk_count <= clk_count+1'b1;
		
	always @(posedge clk)
		//out_reg<=0;
		if(clk_count>=local_input || rst==1) out<=0;
		else             out<=1;
		
//		//if(clk_count<local_input || rst==1) out<=0;
//		if(clk_count>=local_input || rst==1) out<=0;
//		else             out<=1;

endmodule 