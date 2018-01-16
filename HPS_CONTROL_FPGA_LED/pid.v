module PID
(output signed [31:0] u_out, // output
input signed [31:0] e_in, // input
input clk,
input reset,
input [7:0] k1,
input [7:0] k2,
input [7:0] k3);

	//parameter k1=107; // change these values to suit your system
	//parameter k2 = 104;
	//parameter k3 = 2;
	//note:
	//k1 = kp+ki+kd
	//k2 = -kp-2kd (input is positive though)
	//k3 = kd
	
	reg signed [31:0] u_prev;
	reg signed [31:0] e_prev[0:1];
	reg signed [31:0] e_in_buffered;
	
	assign u_out = u_prev + k1*e_in_buffered - k2*e_prev[0] + k3*e_prev[1];

	always @(posedge clk)
	begin
		e_in_buffered <= e_in;
		if (reset == 1) 
		begin
			u_prev <= 0;
			e_prev[0] <= 0;
			e_prev[1] <= 0;
		end
		else 
		begin 
			e_prev[1] <= e_prev[0];
			e_prev[0] <= e_in_buffered;
			u_prev <= u_out;
		end
	end
endmodule 



module PID_testbench;
	reg clk, reset;
	reg [31:0] e_in;
	//reg [7:0] k1;
	//reg [7:0] k2;
	//reg [7:0] k3;
	wire [31:0] u_out;
	
	parameter k1 = 1;
	parameter k2 = 0;
	parameter k3 = 0;
	
	parameter stimDelay = 10;
	
	PID myPID(	u_out,
	e_in,
	clk,
	reset,
	k1,
	k2,
	k3);

	always #1 clk = ~clk;

	initial
	begin
		clk = 0; e_in = 0; reset = 0;
		
		
		#(stimDelay)
		reset = 1;
		#(stimDelay);
		reset = 0;
		
		repeat(10)
		begin
			#(stimDelay) e_in = e_in + 1;
		end
		
		
		#100; //Let simulation finish
	end
endmodule