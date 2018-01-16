module quad_counter(clk, quadA, quadB, count, reset);
	input clk, quadA, quadB, reset;
	output [31:0] count;

	reg [2:0] quadA_delayed, quadB_delayed;
	always @(posedge clk) quadA_delayed <= {quadA_delayed[1:0], quadA};
	always @(posedge clk) quadB_delayed <= {quadB_delayed[1:0], quadB};

	wire count_enable = quadA_delayed[1] ^ quadA_delayed[2] ^ quadB_delayed[1] ^ quadB_delayed[2];
	wire count_direction = quadA_delayed[1] ^ quadB_delayed[2];

	reg [31:0] count;
	
	
	always @(posedge clk)
	begin
	  if(reset)
	  begin
			count<=0;
	  end
	  else if(count_enable)
	  begin
		 if(count_direction) count<=count+1; else count<=count-1;
	  end
	end

endmodule 


module quad_counter_testbench;
	reg clk, quadA, quadB, reset;
	wire [31:0] count;
	//signal clk : std_logic := '0';
	
	parameter stimDelay = 50;
	
	quad_counter myCounter(clk, quadA, quadB, count, reset);

	always #1 clk = ~clk;

	initial
	begin
		clk = 0; quadA = 0; quadB = 0; reset = 0;
		
		
		#(stimDelay)
		reset = 1;
		#(stimDelay);
		reset = 0;
		
		repeat(100000000)
		begin
			#(stimDelay) quadB = 1;
			#(stimDelay) quadA = 1;
			#(stimDelay) quadB = 0;
			#(stimDelay) quadA = 0;
		end
		
		#(stimDelay)
		reset = 1;
		#(stimDelay);
		reset = 0;
		
		#(stimDelay) quadB = 0;
		#(stimDelay) quadA = 0;
		#(stimDelay) quadB = 1;
		#(stimDelay) quadA = 1;
		#(stimDelay) quadB = 0;
		#(stimDelay) quadA = 0;	
	
		#(stimDelay) quadA = 1;
		#(stimDelay) quadB = 1;
		#(stimDelay) quadA = 0;
		#(stimDelay) quadB = 0;
		#(stimDelay) quadA = 1;
		#(stimDelay) quadB = 1;
	
		
		#100; //Let simulation finish
	end
endmodule