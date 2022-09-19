
// RX
// Copyright 2010 University of Washington
// License: http://creativecommons.org/licenses/by/3.0/
// 2008 Dan Yeager

// RX module converts EPC Class 1 Gen 2 time domain protocol
// to a serial binary data stream
// Sample bitout on bitclk positive edges.

// TR cal measurement provided for TX clock calibration.
// RT cal for debugging purposes and possibly adjustment
//   of oscillator frequency if necessary.

module rx (reset, clk, demodin, bitout, bitclk, rx_overflow_reset, trcal, rngbitout,count);

  input reset, clk, demodin;
  output bitout, bitclk, rx_overflow_reset, rngbitout;
  output [9:0] trcal;
  output wire[9:0] count;

  reg bitout, bitclk, rngbitout;
  reg [9:0] trcal, rtcal;

  // States, first 4 are for preamble; state_bits for getting bits.
  parameter STATE_DELIMITER = 5'd0;
  parameter STATE_DATA0     = 5'd1;
  parameter STATE_RTCAL     = 5'd2;
  parameter STATE_TRCAL     = 5'd4;
  parameter STATE_BITS      = 5'd8;
  reg [4:0] commstate; // command state, which command are we on right now? --> that state
  
  parameter STATE_WAIT_DEMOD_LOW  = 4'd0;
  parameter STATE_WAIT_DEMOD_HIGH = 4'd1;
  parameter STATE_EVAL_BIT        = 4'd2;
  parameter STATE_RESET_COUNTER   = 4'd4;
  reg [3:0] evalstate; // evaluating the pie symbols we got
  
   // 10 bit counter
  
  // inverting the clock, for negedge triggering?
  wire clkb;
  assign clkb = ~clk;
  
  reg counterreset, counterenable;
  wire overflow;
  
  wire counter_reset_in;
  assign counter_reset_in = counterreset|reset;
  
  //calling the 10 bit counter module
  //module counter10 (clk, reset, enable, count, overflow);
  //If counter exceeds rtcal in BITS state, rx_overflow_reset is set
  counter10 counter (clkb, counter_reset_in, counterenable, count, overflow);
  assign  rx_overflow_reset = overflow | ((commstate == STATE_BITS) && (count > rtcal));
  
always @ (posedge clk or posedge reset) begin
     
if( reset ) begin
    bitout <= 0;
    bitclk <= 0;
    rtcal  <= 10'd0;
    trcal  <= 10'd0;
    rngbitout     <= 0;
    counterreset  <= 0;
    counterenable <= 0;
    commstate <= STATE_DELIMITER; // initially looking for delimiter
    evalstate <= STATE_WAIT_DEMOD_LOW; // waiting to get a low, which is the delimiter. By default in high state if no data coming in

// process normal operation
end else begin

  if(evalstate == STATE_WAIT_DEMOD_LOW) begin
    if (demodin == 0) begin
      evalstate <= STATE_WAIT_DEMOD_HIGH; //now that we got low (demodin =0), we will be waiting for a high next
      if(commstate != STATE_DELIMITER) counterenable <= 1; // counter is for checking length, and length checked for everything but the delimiter
      else                             counterenable <= 0;
      counterreset <= 0;
    end

  end else if(evalstate == STATE_WAIT_DEMOD_HIGH) begin
    if (demodin == 1) begin
      evalstate     <= STATE_EVAL_BIT;//now that we got a high, will be evluating bits next
      counterenable <= 1; // counter enabled here
      counterreset  <= 0; // next bits eval so counter has to be reset, but in the next clock cycle
    end
    
  //reset the counter at every rising edge of demodin, clear the bit clock too (Bit clock = rising edges of Demodin)
//wherever rising edge, bit must be evaluated   
  end else if(evalstate == STATE_EVAL_BIT) begin
    counterreset <= 1;
    evalstate    <= STATE_RESET_COUNTER;
    bitclk       <= 0;
    rngbitout    <= count[0]; //using the last bit of counter as the 
    
    case(commstate)
      STATE_DELIMITER: begin
        commstate <= STATE_DATA0;
      end
      STATE_DATA0: begin
        commstate <= STATE_RTCAL;
      end
      STATE_RTCAL: begin
        rtcal <= count;
        commstate <= STATE_TRCAL;
      end
      STATE_TRCAL: begin
        if(count > rtcal) begin
          trcal  <= count;
        end else if(count[8:0] > rtcal[9:1]) begin // divide rtcal by 2
          bitout <= 1;
          commstate <= STATE_BITS;
        end else begin
          bitout <= 0;
          commstate <= STATE_BITS;
        end
      end
      STATE_BITS: begin
         if(count[8:0] > rtcal[9:1]) begin // data 1 (divide rtcal by 2)
           bitout <= 1;
         end else begin // data 0
           bitout <= 0;
         end
      end
      default: begin
        counterreset <= 0;
        commstate    <= 0;
      end
    endcase // case(mode)
    
    
  end else if(evalstate == STATE_RESET_COUNTER) begin
    if(commstate == STATE_BITS) begin
      bitclk <= 1;
    end
    counterreset <= 0;
    evalstate    <= STATE_WAIT_DEMOD_LOW;
      
  end else begin // unknown state, reset.
    evalstate <= 0;
  end
end // ~reset
end // always @ clk

endmodule // 
