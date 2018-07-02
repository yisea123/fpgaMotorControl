module heartbeat(
output [31:0] output_reset,
input clk_50Mhz,
input [31:0] rst,
output gpio_out100hz,
output hbeat_out,
input [31:0] beat,
input e_stop
);

	// generate 100 Hz from 50 MHz, bit flip at 200hz is equivalent to 100hz clock(square wave)
	reg [17:0] count_reg = 0; // 18 bit counter to 25Mhz
	reg out_100hz = 0;
	
	//Declarations for heartbeat
	reg beat_last = 0; 
	reg initial_loop = 1;
	reg beat_check = 0;
	reg [7:0] counter = 150;
	reg [31:0] temp_rst = 32'hFFFFFFFF;

	assign output_reset = temp_rst;
	assign gpio_out100hz = out_100hz;
	assign hbeat_out = beat_check;

	
	//Downsample clock
	always @(posedge clk_50Mhz) 
	begin
		if (count_reg < 2499) begin
			count_reg <= count_reg + 18'h1; //18 bit decimal 1
		end 
		else begin
			count_reg <= 0;
			out_100hz <= ~out_100hz;//this is now 10000hz
		end
	end
	

	//Heartbeat
	always @(posedge out_100hz) 
	begin
		if ((beat_last != beat[0]) && (initial_loop == 0)) begin
			counter <= 0;
			beat_check <= !beat_check;
			//temp_rst <= 0;
			if (e_stop == 1)
				temp_rst <= 32'hFFFFFFFF;
			else
				temp_rst <= 0|rst; //bitwise or
		end
		else begin
			counter <= counter + 1;
			if (counter >= 150) begin
				temp_rst <= 32'hFFFFFFFF;
				counter <= 8'h96; //150 decimal
				initial_loop <= 0;
			end
		end
		beat_last <= beat[0];
	end
	
	
endmodule
