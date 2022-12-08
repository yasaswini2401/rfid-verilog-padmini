//modified, iter1

///for read sensor data, procedure same as read?

`timescale 1ns/1ns

// PACKET PARSE 
// Copyright 2010 University of Washington
// License: http://creativecommons.org/licenses/by/3.0/
// 2008 Dan Yeager

// This module parses the received packets.
// Currently it supports:

// Handle checking for ack, req_rn, read, write (not kill).
// This tells us if the reader requested handle matches our own.
// The last bit for ACK is tricky because the controller wants to know
// as soon as the last bit clock edge occurs if the handle passed and
// ack doesn't have any bits after the requested handle to clock this module.
// So, there is a special case for the last bit of ack.

// todo: EBV parsing, An extensible bit vector (EBV) is a data structure with an extensible data range
//This protocol uses EBV-8 values to represent memory addresses and mask lengths.

// todo: PTR parsing for read and write.

// todo: Data decoding for write.

module packetparse(reset, bitin, bitinclk, packettype, //inputs
                   rx_q, rx_updn,
                   currenthandle, currentrn, //inputs as well
                   handlematch, readwritebank, readwriteptr, readwords,
                   writedataout, writedataclk,
                   /// for transmit clk, need calibrate, freq select and rforb - will send to controller
                   pllenab, freq_channel, rfob,
                   //// for sample sens data
                   senscode,
                   ///// for read sample data
                   morb_trans);
                   
//6 inputs
input reset, bitin, bitinclk;
///
input [11:0]  packettype;

input [15:0] currenthandle;
input [15:0] currentrn;

//8 outputs
output handlematch;
output [3:0] rx_q;
output [2:0] rx_updn;

///
output rfob;
output pllenab;
output [3:0] freq_channel;
////
output [2:0] senscode;
/////
output morb_trans; //main or backscatter transmitter


output [1:0] readwritebank; //which memory bank to read from, if read. or write to, if write
output [7:0] readwriteptr; //first word pointer?
output [7:0] readwords;//number of words to read
output writedataout, writedataclk; //parses the write data, which is included in the packet, if its a write command

//op registers or wires
reg       handlematch;
reg [3:0] rx_q;
reg [2:0] rx_updn;
reg [1:0] readwritebank;
reg [7:0] readwriteptr;
reg [7:0] readwords;
  
reg writedataout;
wire writedataclk;

/// treat in same way as rx_q
reg rfob;
reg pllenab;
reg [3:0] freq_channel;
////
reg [2:0] senscode;
/////
reg morb_trans;
  
//created registers

reg [3:0] handlebitcounter; // handle is 16 bits, so 4 bit counter needed
reg [5:0] bitincounter; // 6 bit counter
reg       matchfailed;

//read and write
reg       writedataen;
reg       writedataengated;
reg       bankdone;
reg       ptrdone;
reg       ebvdone;
reg       datadone;

assign writedataclk = bitinclk & writedataengated;

/*
// check functionality by intentionally breaking with fake rn
wire [15:0] fixedrn;
assign fixedrn = 16'h0001;
wire thisbitmatches;
assign thisbitmatches = (bitin == fixedrn[~handlebitcounter]);
assign bit1matches    = (bitin == fixedrn[14]);
*/

// real rn
wire thisbitmatches;
assign thisbitmatches = (bitin == currenthandle[~handlebitcounter]); // handlebitcounter is a 4 bit counter

wire lastbit;
assign lastbit  = (handlebitcounter == 15); //is a flag - checks as high if counter reached end of 16 states

//handlematch is 1 (OR) counter reached end, cmd is ACK and this bit matches
wire handlematchout;
assign handlematchout = handlematch | 
       ((handlebitcounter == 15) && packettype[1] && thisbitmatches);
  
//the 12 bit command names, rx_cmd or packettype
///
  parameter ACK        = 12'b000000000010;
  parameter QUERY      = 12'b000000000100;
  parameter QUERYADJ   = 12'b000000001000;
  parameter REQRN      = 12'b000001000000;
  parameter READ       = 12'b000010000000;
  parameter WRITE      = 12'b000100000000;
  ///                     2  0
  parameter TRNS       = 12'b001000000000;
  parameter SAMPSENS   = 12'b010000000000;
  parameter SENSDATA   = 12'b100000000000;
  

// gate the write-data clk en to the negative edges
// so it doesn't glitch.
always @ (negedge bitinclk or posedge reset) begin
  if (reset) begin
    writedataengated <= 0;
  end else begin
    writedataengated <= writedataen; // by gated, just mean shifted a little earlier to avoid glitches cus of prop delay
  end
end

always @ (posedge bitinclk or posedge reset) begin
  if (reset) begin
  //for ack
    handlebitcounter <= 0;
    bitincounter     <= 0;
    handlematch      <= 0;
    matchfailed      <= 0;
    
  //read and write
    bankdone         <= 0;
    ptrdone          <= 0;
    ebvdone          <= 0;
    datadone         <= 0;

    writedataen      <= 0;
    writedataout     <= 0;
  //for query, query adj
    rx_q             <= 4'd0;
    rx_updn          <= 3'd0;
//read and write
    readwritebank    <= 2'd0;
    readwriteptr     <= 8'd0;
    readwords        <= 8'd0;
    ///
    pllenab          <= 0;
    freq_channel     <= 4'd0;
    rfob             <= 0; //by default will be either baseband or transmitter frequency
    ////
    senscode         <= 3'd0; //sensor code

// check for read, write, req_rn, ack
  end else begin
    case(packettype)
      QUERY: begin
        //basically, first 8 bits are already read. ptr is just to indicate that 8 bit word was read.
        //Also, 4 bits have already been pushed into packet parser, but packettype had not beenreceived by then
        //So, thats where first 4 bits are going
        if (!ptrdone) begin
          if (bitincounter >= 8) begin  // ptr done, bit counter is for 9 bits long(0 to 8)
            ptrdone      <= 1;
            bitincounter <= 0;
          end else begin
            bitincounter <= bitincounter + 6'd1;
          end
        // read the 4 q bits
        end else if (bitincounter == 0) begin
          bitincounter <= bitincounter + 6'd1;
          rx_q[3] <= bitin;
        end else if (bitincounter == 1) begin
          bitincounter <= bitincounter + 6'd1;
          rx_q[2] <= bitin;
        end else if (bitincounter == 2) begin
          bitincounter <= bitincounter + 6'd1;
          rx_q[1] <= bitin;
        end else if (bitincounter == 3) begin
          bitincounter <= bitincounter + 6'd1;
          rx_q[0] <= bitin;
        end
      end
      QUERYADJ: begin
        if (!ptrdone) begin
          if (bitincounter >= 1) begin  // ptr done
            ptrdone      <= 1;
            bitincounter <= 0;
          end else begin
            bitincounter <= bitincounter + 6'd1;
          end
        // read the 3 up/dn bits
        end else if (bitincounter == 0) begin
          bitincounter <= bitincounter + 6'd1;
          rx_updn[2] <= bitin;
        end else if (bitincounter == 1) begin
          bitincounter <= bitincounter + 6'd1;
          rx_updn[1] <= bitin;
        end else if (bitincounter == 2) begin
          bitincounter <= bitincounter + 6'd1;
          rx_updn[0] <= bitin;
        end
      end
      ACK: begin
        if (!matchfailed && !lastbit && thisbitmatches) begin
          handlebitcounter <= handlebitcounter + 4'd1;
        // check the last bit of the handle
        end else if (!matchfailed && lastbit && thisbitmatches) begin
          handlematch <= 1;
        end else if (!handlematch) begin
          matchfailed <= 1;
        end
      end
      REQRN: begin
        if (!matchfailed && !lastbit && thisbitmatches) begin
          handlebitcounter <= handlebitcounter + 4'd1;
        // check the last bit of the handle
        end else if (!matchfailed && lastbit && thisbitmatches) begin
          handlematch <= 1;
        end else if (!handlematch) begin
          matchfailed <= 1;
        end
      end
      READ: begin
        // read: we start comparing after ebv is done and words is done.
        if (!bankdone && (bitincounter == 0)) begin
          bitincounter <= bitincounter + 6'd1;
          readwritebank[1] <= bitin;
        end else if (!bankdone && (bitincounter >= 1)) begin
          bankdone <= 1;
          bitincounter <= 0;
          readwritebank[0] <= bitin;

        // if bitctr==0, see if the ebv is done
        end else if (!ebvdone && (bitincounter == 0)) begin
          if (!bitin) begin  // last ebv byte indicator
            ebvdone        <= 1;
            bitincounter   <= 1;
          end else begin
            bitincounter <= bitincounter + 6'd1;
          end
        // wait out the ebv if its not done
        end else if (!ebvdone && (bitincounter < 7)) begin
          bitincounter <= bitincounter + 6'd1;
        // restart to the next ebv byte
        end else if (!ebvdone && (bitincounter >= 7)) begin
          bitincounter <= 0;

        // read in the last 7 ebv byte bits as ptr
        end else if (!ptrdone) begin
          if (bitincounter >= 7) begin  // ptr done
            ptrdone      <= 1;
            bitincounter <= 0;
          end else begin
            bitincounter <= bitincounter + 6'd1;
          end
          readwriteptr[~bitincounter[2:0]] <= bitin;

        // read the 8 word bits
        end else if (ptrdone && (bitincounter < 8)) begin
          bitincounter <= bitincounter + 6'd1;
          readwords[~bitincounter[2:0]] <= bitin;

        // check the first and middle bits of the handle
        end else if (!matchfailed && !lastbit && thisbitmatches) begin
          handlebitcounter <= handlebitcounter + 4'd1;
        // check the last bit
        end else if (!matchfailed && lastbit && thisbitmatches) begin
          handlematch <= 1;
        end else if (!handlematch) begin
          matchfailed <= 1;
        end
      end
      WRITE: begin
        // write: we start comparing after ebv is done and data is done.
        if (!bankdone && (bitincounter == 0)) begin
            bitincounter <= bitincounter + 6'd1;
            readwritebank[1] <= bitin;
        end else if (!bankdone && (bitincounter >= 1)) begin
            bankdone <= 1;
            bitincounter <= 0;
            readwritebank[0] <= bitin;
        
        // if bitctr==0, see if the ebv is done
        end else if (!ebvdone && (bitincounter == 0)) begin
            if (!bitin) begin  // last ebv byte indicator
                ebvdone      <= 1;
                bitincounter <= 1;
            end else begin
                bitincounter <= bitincounter + 6'd1;
            end
        // wait out the ebv if its not done
        end else if (!ebvdone && (bitincounter < 7)) begin
            bitincounter <= bitincounter + 6'd1;
        // restart to the next ebv byte
        end else if (!ebvdone && (bitincounter >= 7)) begin
            bitincounter <= 0;
            
        // read in the last 7 ebv byte bits as ptr
        end else if (!ptrdone) begin
            if (bitincounter >= 7) begin  // ptr done
                ptrdone      <= 1;
                bitincounter <= 0;
                writedataen  <= 1;
            end else begin
                bitincounter <= bitincounter + 6'd1;
            end
            
            readwriteptr[~bitincounter[2:0]] <= bitin;
            
        // get the 16 data bits
        end else if (!datadone) begin
            if (bitincounter >= 15) begin  // data done
                datadone     <= 1;
                bitincounter <= 0;
                writedataen  <= 0;
            end else begin
                bitincounter <= bitincounter + 6'd1;
            end
            
            writedataout <= bitin ^ currentrn[~bitincounter[3:0]];
            
        // check the first and middle bits of the handle
        end else if (!matchfailed && !lastbit && thisbitmatches) begin
            handlebitcounter <= handlebitcounter + 4'd1;
        // check the last bit
        end else if (!matchfailed && lastbit && thisbitmatches) begin
            handlematch <= 1;
        end else if (!handlematch) begin
            matchfailed <= 1;
        end else begin
            writedataen <= 0;
        end
      end
      ///
      TRNS: begin
          if (bitincounter == 0) begin
                bitincounter <= bitincounter + 6'd1;
                pllenab <= bitin;
          end else if (bitincounter == 1) begin
                bitincounter <= bitincounter + 6'd1;
                freq_channel[3] <= bitin;
          end else if (bitincounter == 2) begin
                bitincounter <= bitincounter + 6'd1;
                freq_channel[2] <= bitin;
          end else if (bitincounter == 3) begin
                bitincounter <= bitincounter + 6'd1;
                freq_channel[1] <= bitin;
          end else if (bitincounter == 4) begin
                bitincounter <= bitincounter + 6'd1;
                freq_channel[0] <= bitin;
          end else if (bitincounter == 5) begin//last, 6th count since 6 bits to be parsed
                bitincounter <= bitincounter + 6'd1;
                rfob <= bitin;
          end
            
      end
      SAMPSENS: begin
          // samplesens
            if (bitincounter == 0) begin
                  bitincounter <= bitincounter + 6'd1;
                  senscode[2] <= bitin;
            end else if (bitincounter == 1) begin
                  bitincounter <= bitincounter + 6'd1;
                  senscode[1] <= bitin;
            end else if (bitincounter == 2) begin
                  bitincounter <= bitincounter + 6'd1;
                  senscode[0] <= bitin;
            end
    /*
            // check the first and middle bits of the handle, for rn16
            end else if (!matchfailed && !lastbit && thisbitmatches) begin
              handlebitcounter <= handlebitcounter + 4'd1;
            // check the last bit
            end else if (!matchfailed && lastbit && thisbitmatches) begin
              handlematch <= 1;
            end else if (!handlematch) begin
              matchfailed <= 1;
            end
      */
          
      end
      SENSDATA:begin
            if (bitincounter == 0) begin
                  bitincounter <= bitincounter + 6'd1;
                  morb_trans <= bitin;
            end else if (bitincounter == 1) begin
                  bitincounter <= bitincounter + 6'd1;
                  senscode[2] <= bitin;
            end else if (bitincounter == 2) begin
                  bitincounter <= bitincounter + 6'd1;
                  senscode[1] <= bitin;
            end else if (bitincounter == 3) begin
                  bitincounter <= bitincounter + 6'd1;
                  senscode[0] <= bitin;
            // check the first and middle bits of the handle, for rn16
            end else if (!matchfailed && !lastbit && thisbitmatches) begin
              handlebitcounter <= handlebitcounter + 4'd1;
            // check the last bit
            end else if (!matchfailed && lastbit && thisbitmatches) begin
              handlematch <= 1;
            end else if (!handlematch) begin
              matchfailed <= 1;
            end 
            
      end
      default begin // do nothing. 
        // either cmd is not yet received or we don't need to check handle.
      end
    endcase
  end // ~reset
end // always

endmodule

