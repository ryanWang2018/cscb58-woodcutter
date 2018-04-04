// Part 2 skeleton

module woodcutter
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
        KEY,
        SW,
		LEDR,
        HEX0,
        HEX1,
		  HEX4,
		  HEX5,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B   						//	VGA Blue[9:0]
	);

	input			CLOCK_50;				//	50 MHz
	input   [9:0]   SW;
	input   [3:0]   KEY;
	output  [17:0]  LEDR;
    output  [6:0]   HEX0, HEX1, HEX4, HEX5;
	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
	
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	
	//wire writeEn;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(1'b1),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
    // Create the colour, x, y and writeEn wires that are inputs to the controller.
	
    
    //-----------------------------------------------------------------------------
    // declear all wire reg here 
    wire branch_sig, char_sig, over_sig;
//    wire [4:0] b_index;
	wire print_branch, print_char, check, score_check;
    wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
    wire resetn;
    wire go;
    wire left, right;
    wire [3:0] h0, h1;
	 //-------------------------------
	 wire dividerout;
	wire [3:0]sync_counter_out;
	wire [3:0]sync_counter_out2;
	wire timeup;
	wire finish;
	wire time_begin;
    //-----------------------------------------------------------------------------
    // assign all the sw and key
	assign resetn = KEY[1];
    assign left = !KEY[3];
    assign right = !KEY[2];
    assign go = (left ^ right);
    //assign finish = over_sig | timeup;
    //-----------------------------------------------------------------------------
    datapath da(
                .clk(CLOCK_50),
				.left(left),
				.right(right),
                .resetn(resetn),
                .print_branch(print_branch),
				.print_char(print_char),
                .check(check),
				.score_check(score_check),
                .x(x),
                .y(y),
                .branch_sig(branch_sig),
				.char_sig(char_sig),
                .over_sig(over_sig),
                .colour(colour),
				.dig10(h1),
				.dig1(h0));
    
    // over_sig depends on both timer and checker (or)
    // add timmer here :
	 timer(.resetn(resetn), .go(go), .over_sig(over_sig), .time_up(timeup), .time_begin(time_begin), .finish(finish));
	 
	 RateDivider d1(.enable(1'b1), .resetn(resetn), .clk_out(dividerout), .clk(CLOCK_50));
	
	 sync_counter s1(.clock(dividerout), .resetn(resetn), .start(time_begin), .clkout(sync_counter_out), .clkout2(sync_counter_out2), .done(timeup));
	
  	hex_display hex4(.IN(sync_counter_out), .OUT(HEX4[6:0]));
	
	hex_display hex5(.IN(sync_counter_out2), .OUT(HEX5[6:0]));
    
    control con(
            .resetn(resetn),
            .clk(CLOCK_50),
            .go(go),
            .branch_sig(branch_sig),
            .char_sig(char_sig),
            .over_sig(finish),
            
            .print_branch(print_branch),
            .print_char(print_char),
            .check(check),
			.score_check(score_check));
	assign LEDR[0] = finish;
    hex_display hex1(.IN(h1),
                     .OUT(HEX1));
                        
    hex_display hex0(.IN(h0),
                     .OUT(HEX0));
    
endmodule

module timer(resetn, go, over_sig, time_up, time_begin, finish);
	input resetn, go, over_sig, time_up;
	output reg time_begin, finish;
	always@(*)
	 begin
		if (!resetn)
		begin
			time_begin = 1'b0;
			finish = 1'b0;
		end
		else if (over_sig == 1'b1 | time_up == 1'b1)
		begin
			time_begin = 1'b0;
			finish = 1'b1;
		end
		else if (go == 1'b1)
		begin
			time_begin = 1'b1;
		end
		
	 end
endmodule

module datapath(
    input clk,
	input left,
	input right,
    input resetn,
    input print_branch,
	input print_char,
    input check,
	 input score_check,
     
    output [7:0] x,
    output [6:0] y,
    output branch_sig,          // move to the s_draw_char
	output char_sig,            // move to the s_check
    output over_sig,
    output [2:0] colour,
	 output [3:0] dig10,
	 output [3:0] dig1
    );
	
    //------------------------------------------------------
    // declear wire reg here
    wire [7:0] x0, x1;
	wire [6:0] y0, y1;
	wire [2:0] colour0, colour1;
    wire [1:0] b_pos;               // 2'b01(right), 2'b00(none), 2'b10(left)
	 wire hit_sig;
    //------------------------------------------------------
    
	
	assign x = print_branch*x0 + print_char*x1;
	assign y = print_branch*y0 + print_char*y1;
	assign colour = print_branch*colour0 + print_char*colour1;
	 
    Branch my_branches(clk, print_branch, resetn, x0, y0, colour0, branch_sig, b_pos);
	Character my_char(clk, print_char, left, right, resetn, x1, y1, colour1, char_sig); 
	CheckCollision cc(clk, resetn, check, b_pos, left, right, hit_sig);
	score sc(clk, resetn, score_check, hit_sig, dig10, dig1);
    assign over_sig = hit_sig;
    // store the lowest branches [1:0] at a register
    // store the character position at a register
    // pass they into the check hit position return a over_sig
    // pass over_sig to controller move to over state
endmodule


module control(
    input resetn,
    input clk,
    input go,
    input branch_sig,
    input char_sig,
    input over_sig,
    
    output reg print_branch,
    output reg print_char,
    output reg check,
	output reg score_check
    );
    
    // add s_check and s_over
    // s_check => s_over if over_sig =1'b1
    // s_over => s_init automatically
    
	localparam s_init = 5'd0,          
               s_ready = 5'd1,
               s_draw_branch = 5'd2,
               s_draw_char = 5'd3,
               s_wait = 5'd4,
               s_check = 5'd5,
               s_over = 5'd6,
			   s_score = 5'd7;
	reg [2:0] curr, next;
	
    always@(*)
    begin: states_table
        case (curr)
            s_init: next = s_draw_char;
            s_ready: next = go ? s_draw_char : s_ready;
            s_draw_char : next = char_sig ? s_draw_branch : s_draw_char;
			s_draw_branch: next = branch_sig ? s_check : s_draw_branch;
            s_check: next = over_sig ? s_over : s_score;
			s_score: next = s_wait;
            s_over: ;
            s_wait: next = go ? s_wait : s_ready;
		endcase
    end	
	 
	 // output logic
    always@(*)
    begin: enable_signals
        print_branch = 1'b0;
        print_char = 1'b0;
        check = 1'b0;
		score_check = 1'b0;
        case (curr)
            s_init:
            begin
                print_branch = 1'b0;
                print_char = 1'b0;
                check = 1'b0;
                score_check = 1'b0;
            end
            
            s_draw_branch:
            begin
				print_branch = 1'b1;	
//				print_char = 1'b0;
            end
				
			s_draw_char:
			begin
//				print_branch = 1'b0;
				print_char = 1'b1;
			end
            s_check:
                check = 1'b1;
			s_score:
				score_check = 1'b1;
					 
		endcase
    end
	always @(posedge clk)
    begin
        if(resetn == 1'b0)
            curr <= s_init;   // reset to s_init
        else
            curr <= next;  
    end 
endmodule  

module CheckCollision(clk, resetn, check, b_pos, left, right, over_sig);
    input clk, resetn, check;
    input [1:0] b_pos;
    input left, right;
    output reg over_sig;
    
    always@(negedge clk)
    begin
        if (!resetn)
            over_sig = 1'b0;        //difference from backup_version
        else if (check == 1'b1)
            begin
            if (b_pos == 2'b10 && left == 1'b1)         // got hit at the left
                over_sig <= 1'b1;
            else if (b_pos == 2'b01 && right == 1'b1)          //got hit at the right 
                over_sig <= 1'b1;
            else
                over_sig <= 1'b0;
            end
    end
endmodule

module draw_branch(input clk,
				   input resetn,
                   input [7:0]x,
                   input [6:0]y, 
                   input start,
                   output reg [7:0] x_out,
                   output reg [6:0] y_out,
                   output reg done);

    reg [5:0] counter;
    reg [3:0] counter2;
 
	always @(negedge clk) 
	begin
		if (!resetn)
		begin
			counter <= 6'b000000;		//x
			counter2 <= 4'b0000;      //y
			done <=1'b0;
		end
		else if(start == 1'b1)
		begin 
			done <=1'b0;
			if(counter2 > 4'b0101)
			begin
				// this need to change to an output stop signal
				counter2 <= 4'b0;
				counter <= 6'b0;
				done <= 1'b1;
			end
			else if(counter[5:0] < 6'b101000)
			begin
				x_out <= x + counter[5:0];
				y_out <= y + counter2[3:0];
				
			end

			else if(counter[5:0] == 6'b101000)
			begin
				counter2 <= counter2 + 1'b1;
				counter <= 6'b000000;
			end
			counter <= counter + 1'b1;

		end
	end
endmodule

module Branch(clk, drawBranch, reset, x, y, colour, nextState, b_pos);
    input clk, drawBranch, reset;
    output reg [7:0] x ;
    output reg [6:0] y ;
    output reg [2:0] colour;
    output reg nextState;
    output reg [1:0] b_pos;
    
    // ------------------------ local ----------------------------------
    wire [99:0] branchLeft, branchRight;
    reg [99:0] leftTemp, rightTemp;
    reg [4:0] index;
	reg [4:0] counter;
	
	reg [5:0] counter_x;
	reg [3:0] counter_y;
	reg next_branch;
    //  ----------------value assignment--------------------------------
    
    assign branchLeft = 100'b0010010010010010010010010010010010010010010010010010010010010010010010010010010010010010010010010010;   
    assign branchRight = 100'b0001101101000101000100001001000100100001000100001101001100000001100101000001100100001001001100100101;
    
    // -----------------------------------------------------------------
    always @(negedge clk) begin
        if (!reset) 
        begin
            x = 8'b00000000; 
            y = 7'b0000000;
            leftTemp <= branchLeft;
            rightTemp <= branchRight;
            nextState <= 1'b0;
            colour <= 3'b000;
            index <= 5'd0;
            counter <= 5'd0;
            b_pos <= 2'b00;
				
				counter_x = 6'b000000;
				counter_y = 4'b0000;
				next_branch <=1'b0;
        end
        else if (drawBranch == 1'b1)
        begin                       // get the branch position (x y)
            if (index < 5)
                x = 8'd25;
            else
                x = 8'd96;
            case (index)
                4'd4: y = 7'd10;
                4'd3: y = 7'd34;
                4'd2: y = 7'd58;
                4'd1: y = 7'd82;	
                4'd0: y = 7'd106;
                4'd9: y = 7'd10;
                4'd8: y = 7'd34;
                4'd7: y = 7'd58;
                4'd6: y = 7'd82;
                4'd5: y = 7'd106;
            endcase
            if (counter < 10)
				begin
					if (index < 5)              // check branchLeft for colour
					begin
						if (leftTemp[99-index] == 1'b1)          // check the first bit in the leftTemp for colour
							colour <= 3'b100;    
						else
							colour <= 3'b000;
					end
					else if (index > 4 && index < 10)        // check branchRight for colour
					begin 
						if (rightTemp[104-index] == 1'b1)
							colour <= 3'b100;    
						else
							colour <= 3'b000;
					end
					// ------------------------------------draw single branch
					if (next_branch == 1'b1)
					begin 
					counter <= counter + 1;
					index <= index + 1;
					next_branch = 1'b0;
					end
					else
					begin
						next_branch <=1'b0;
						if(counter_y > 4'b0101)
						begin
						// this need to change to an output stop signal
						counter_y = 4'b0;
						counter_x = 6'b0;		
						next_branch <= 1'b1;
						end
						else if(counter_x[5:0] < 6'b101000)
						begin
						x = x + counter_x[5:0];
						y = y + counter_y[3:0];	
						end

						else if(counter_x[5:0] == 6'b101000)
						begin
						counter_y = counter_y + 1'b1;
						counter_x = 6'b000000;
						end
						counter_x = counter_x + 1'b1;
						end
						//--------------------------------------------------
					nextState <= 1'b0;
					
                end
					 
            else
            begin
                b_pos <= {leftTemp[99], rightTemp[99]};
                index = 0;
                counter <= 0;
                nextState <= 1'b1;
                leftTemp = leftTemp << 1'b1;
                rightTemp = rightTemp << 1'b1;
            end
        end
        
    end
endmodule

module Character(clk, drawChar, left, right, reset, x, y, colour, nextState);
    input clk, drawChar, left, right, reset;
    output reg [7:0] x;
    output reg [6:0]y;
	 output reg [2:0] colour;
    output reg nextState;
    
    // ---------------------------------------------------------------
    reg [1:0] counter;
	 reg [5:0] counter_y;
	 reg [4:0] counter_x;
	 reg next_pos;
    // ---------------------------------------------------------------
    always @(negedge clk) begin
        if (!reset) 
        begin
            x = 8'd40; 
            y = 7'd106;
            colour <= 3'b111;
            nextState <= 1'b0;
            counter = 0;
				
				counter_y = 6'b000000;
				counter_x = 5'b00000;
				next_pos = 1'b0;
        end
        else if (drawChar == 1'b1)
        begin
				y = 7'd106;
				nextState = 1'b0;
				
				if (counter > 1)
				begin
					nextState = 1'b1;
					counter = 0;
				end
            else if (counter == 0)                   // draw left position 
				begin
					x = 8'd40;
					if (left == 1'b1)
                        begin
						colour <= 3'b111;
                        end
					else 
                        begin
						colour <= 3'b000;
                        end
				end
				else if (counter == 1)              // draw right position 
				begin
					x = 8'd100;
					if (right == 1'b1)
                        begin
                        colour <= 3'b111;
                        end
					else 
                        begin
                        colour <= 3'b000;
                        end
				end
				if (next_pos == 1'b1)
				begin
					counter = counter + 1'b1;
					next_pos = 1'b0;
				end
				else
				begin
					next_pos = 1'b0;
					if(counter_x > 5'b10100)
					begin
					// this need to change to an output stop signal
					counter_x = 5'b0;
					counter_y = 6'b0;
					next_pos = 1'b1;
					end
					else if(counter_y[5:0] < 6'b001000)
					begin
					next_pos = 1'b0;
					x = x + counter_y[5:0];
					y = y + counter_x[4:0];
					end

					else if(counter_y[5:0] == 6'b001000)
					begin
					counter_x = counter_x + 1'b1;
					counter_y = 6'b000000;
					end

					counter_y = counter_y + 1'b1;
				end
        end
    end
endmodule


module score(clk, resetn, score_check, over_sig, dig10, dig1);
	input clk, resetn, score_check, over_sig;
	output reg [3:0] dig10, dig1;
	always@(negedge clk)
	begin
		if (!resetn)
		begin
			dig10 <= 4'b0;
			dig1 <= 4'b0;
		end
		else if (score_check == 1'b1 && over_sig == 1'b0)
		begin
			if (dig10 == 4'd9 && dig1 == 4'd9)
			begin
				dig10 <= 4'd0;
				dig1 <= 4'd0;
			end
			else if (dig1 < 4'd9)
				dig1 <= dig1 + 1'd1;
			else 
				begin
					dig1 <= 4'd0;
					dig10 <= dig10 + 4'd1;
				end
		end
	end
endmodule

module hex_display(IN, OUT);
    input [3:0] IN;
	 output reg [7:0] OUT;
	 
	 always @(*)
	 begin
		case(IN[3:0])
			4'b0000: OUT = 7'b1000000;
			4'b0001: OUT = 7'b1111001;
			4'b0010: OUT = 7'b0100100;
			4'b0011: OUT = 7'b0110000;
			4'b0100: OUT = 7'b0011001;
			4'b0101: OUT = 7'b0010010;
			4'b0110: OUT = 7'b0000010;
			4'b0111: OUT = 7'b1111000;
			4'b1000: OUT = 7'b0000000;
			4'b1001: OUT = 7'b0011000;
			4'b1010: OUT = 7'b0001000;
			4'b1011: OUT = 7'b0000011;
			4'b1100: OUT = 7'b1000110;
			4'b1101: OUT = 7'b0100001;
			4'b1110: OUT = 7'b0000110;
			4'b1111: OUT = 7'b0001110;
			
			default: OUT = 7'b0111111;
		endcase

	end
endmodule

module RateDivider(enable, resetn, clk_out, clk);
	input enable, resetn;
	input clk;
	output reg clk_out;
	reg [28:0]counter;
 
	always @(posedge clk)
	begin
		if(~resetn) begin
   		counter <= 28'b0000000000000000000000000000;
   		clk_out <= 0;
		end
		else begin
   		if(counter == 28'b0001011111010111100001000000 && enable == 1'b1) begin
      		counter <= 28'b0000000000000000000000000000;
      		clk_out <= ~clk_out;
   		end
			else if (enable == 1'b1) begin
				counter <= counter + 1;
			end
			else 
				counter <= counter;
		end
	end
endmodule

module sync_counter(clock, resetn, start, clkout, clkout2, done);
	input clock, resetn, start;	

	output reg [3:0]clkout;
	output reg [3:0]clkout2;
	output reg done;
	initial clkout = 4'b1001;
	initial clkout2 = 4'b0001;
	
	reg enable;
	initial enable = 1;
	initial done = 0;

	always@(posedge clock)
	begin
		if(resetn == 1'b0) begin
			clkout <= 4'b1001;
			clkout2 <= 4'b0001;
			enable <= 1;
			done <= 0;
		end
		else if (start == 1'b1)
		begin
			if ((clkout == 4'b0000) && (enable == 1'b1)) 
			begin
				clkout <= 4'b1001;
				done <= 0;
				if (clkout2 == 4'b0000) 
				begin
					done <= 1;
					enable <= 0;
					clkout <= 4'b0000;
				end
				else
					clkout2 <= clkout2 - 1;
			end
			else if (enable == 1'b1)
				clkout <= clkout - 1;
		end
		else // start == 1'b0
		begin
			clkout <= 4'b1001;
			clkout2 <= 4'b0001;
		end
	end
endmodule