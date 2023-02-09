//modified, iter1




// Controller module
// Copyright 2010 University of Washington
// License: http://creativecommons.org/licenses/by/3.0/
// 2008 Dan Yeager

// This is the high level smarts of the RFID tag.
// It decides if and what to send upon a 
// packet complete signal from the rx module.
// If we should transmit, it starts up the tx module
// and waits for it to indicate that it is finished.
// We also return a data select signal (bitsrcselect) to the 'top' module
// which mux'es the epc, rn, and adc into the tx module.

// A couple features have been added for EPC compatibility
// 1. Handle persistence
//    During a Write command, the reader asks for two successive
//    req_rn's.  The first is to be our handle. The second is the
//    write data cover code. We store the handle as our current handle
//    and this condition is kept in a reg tagisopen.
//    For reference, see EPC spec - Annex K
//    
// 2. Q-slotting for TDMA based on the rng
//    Query, QueryAdj and QueryRep commands are used to manage
//    the number of time slots. Query and QueryAdj load
//    the slotcounter with Q bits of the RN from the rng.
//    If slotcounter == 0, tag should TX its RN.
//    QueryRep commands cause tag to decrement slotcounter.
//    This feature is enabled via the use_q input.
//    EPC spec - see Annex J
//    
// 3. Unique ID, removed
//    Tags should have unique ID's (uid).  However, the UID
//    should not change in time unless the reader rewrites the ID.
//    Ying's ID generator has unstable bits, which violates SPEC
//    so we have another option to use a static ID.
//    This feature is enabled via the use_uid input.
//    use_uid=1 -> Ying's ID,    use_uid=0 -> static ID


//the trns command:
//  since we dont have to do encoding or modulation, just for this command - 
//  going to output the bit directly to the bit as a separate output. It
//  won't be an output from demodin.

module controller (reset, clk, rx_overflow, rx_cmd, currentrn, currenthandle,
                   packet_complete, txsetupdone, tx_done, 
                   rx_en, tx_en, docrc, handlematch,
                   bitsrcselect, readfrommsp, readwriteptr, rx_q, rx_updn,
                   use_q, comm_enable,
                   ///
                   plloff,
                   ////
                   adc_sample, 
                   crc5invalid, crc16invalid, sel, sl_flag);
  
  parameter QUERYREP   = 13'b0000000000001;
  parameter ACK        = 13'b0000000000010;
  parameter QUERY      = 13'b0000000000100;
  parameter QUERYADJ   = 13'b0000000001000;
  parameter SELECT     = 13'b0000000010000;
  parameter NACK       = 13'b0000000100000;
  parameter REQRN      = 13'b0000001000000;
  parameter READ       = 13'b0000010000000;
  parameter WRITE      = 13'b0000100000000;
  ///for transmitter wake up bit addition, need to add another bit
  parameter TRNS       = 13'b0001000000000;
  parameter SAMPSENS   = 13'b0010000000000;
  parameter SENSDATA   = 13'b0100000000000;
  parameter BFCONST    = 13'b1000000000000;

  parameter bitsrcselect_RNG = 2'd0;
  parameter bitsrcselect_EPC = 2'd1;
  parameter bitsrcselect_ADC = 2'd2;
  parameter bitsrcselect_UID = 2'd3;
  
  ///
  parameter pll_dur = 150; 
   
  input reset, clk, rx_overflow, packet_complete, txsetupdone, tx_done;
  input [12:0] rx_cmd;
  input [15:0]  currentrn;
  output [15:0] currenthandle;
  output rx_en, tx_en, docrc; // current_mode 0: rx mode, 1: tx mode
  output [1:0] bitsrcselect;
  input [7:0] readwriteptr;
  output readfrommsp;
  input use_q;
  input [3:0] rx_q;
  input [2:0] rx_updn;
  input handlematch, comm_enable;
  
  ///
  output reg plloff;
  ///
  output reg adc_sample;
  input crc5invalid, crc16invalid;
  
  //select functionality
  input [1:0] sel;
  input sl_flag;
  
  
  
  

  reg [3:0] rx_q_reg; //register to store Q value
  reg readfrommsp;
  reg [15:0] storedhandle;
  reg [1:0] bitsrcselect;
  reg docrc; //flag to do crc or not
  reg rx_en, tx_en; //enable transmitting or receiving

  reg commstate;
  parameter STATE_RX      = 1'b0;
  parameter STATE_TX      = 1'b1;
  
  reg [9:0] pllwaitcount;

  // See EPC spec Annex K
  // First request RN sets our handle
  // Second request RN sets the current cover code
  // For write data
  reg tagisopen;
  assign currenthandle = tagisopen ? storedhandle : currentrn;


  // Code to handle Q slotting for time-division multiplexing
  
  // We TX our RN when slot counter == 0 for any of the following commands:
  // They also have special behaviors:
  // Query    -> draw new rn, take Q bits of rn as init slot counter
  // QueryAdj -> draw new rn, adjust stored Q value as per cmd, 
  //             take Q bits of rn as init slot counter (like a query)
  // QueryRep -> decrement existing slot counter value
  
  reg [14:0] slotcounter; 
  // master slot counter WHOAAAAAA
  
  // For query adjust, we will init slot counter based on q_adj
  wire [3:0] q_adj, q_up, q_dn;
  assign q_up  = (rx_q_reg < 4'd15 && rx_updn[2] && rx_updn[1]) ? rx_q_reg + 4'd1 : rx_q_reg;
  assign q_dn  = (rx_q_reg > 4'd0  && rx_updn[0] && rx_updn[1]) ? rx_q_reg - 4'd1 : rx_q_reg;
  assign q_adj = rx_updn[0] ? q_dn : q_up;
  
  // For query, we init slot counter based on rx_q (from the parser module)
  // This code takes Q bits of our rn as the new slot counter.
  // If we get a query or queryAdj, the state machine will 
  //   set slotcounter = newslotcounter as defined here:
  wire [14:0] newslotcounter;
  wire [3:0] q_ctl; // just to store q from the input
  assign q_ctl = (rx_cmd == QUERY) ? rx_q : q_adj;

  reg [14:0] slotcountermask; //this mask will choose which bits actually matter - like a window?

  always @ (q_ctl) begin //when change in q_ctl occurs
    case(q_ctl) //whoaaa since q specifies the mask length basically, we're setting the length with the number of ones
    0:  slotcountermask = 15'b000000000000000;
    1:  slotcountermask = 15'b000000000000001;
    2:  slotcountermask = 15'b000000000000011;
    3:  slotcountermask = 15'b000000000000111;
    4:  slotcountermask = 15'b000000000001111;
    5:  slotcountermask = 15'b000000000011111;
    6:  slotcountermask = 15'b000000000111111;
    7:  slotcountermask = 15'b000000001111111;
    8:  slotcountermask = 15'b000000011111111;
    9:  slotcountermask = 15'b000000111111111;
    10: slotcountermask = 15'b000001111111111;
    11: slotcountermask = 15'b000011111111111;
    12: slotcountermask = 15'b000111111111111;
    13: slotcountermask = 15'b001111111111111;
    14: slotcountermask = 15'b011111111111111;
    15: slotcountermask = 15'b111111111111111;
    default: slotcountermask = 15'b000000000000000;
  endcase
  end
  
  assign newslotcounter = currentrn[14:0] & slotcountermask; // the slot is supposed to have some random Q bit number, Q decided based on mask
  
  
  
  always @ (posedge clk or posedge reset) begin
    if (reset) begin
      commstate <= STATE_RX;
      bitsrcselect    <= 2'd0;
      docrc     <= 0;
      tx_en     <= 0;
      rx_en     <= 0;
      tagisopen <= 0;
      rx_q_reg  <= 0;
      slotcounter  <= 0;
      storedhandle <= 0;
      readfrommsp  <= 0;
      
      ///
      pllwaitcount <= 10'd0;
      plloff <= 0;
      
    end else if (commstate == STATE_TX) begin
        
      if(txsetupdone) begin
        rx_en <= 0;
      end
      if(tx_done) begin // tx_done means transmit done, so receiving starts --> transition to receive state
        tx_en     <= 0;
        commstate <= STATE_RX;
      end else begin
        tx_en <= 1;
      end
    end else if (commstate == STATE_RX) begin  // rx mode, where we finally make sense of the data
        if ((sel == 2'b11 && sl_flag) || (sel == 2'b10 && ~sl_flag) || (sel == 2'b00) || (sel == 2'b01)) begin
            if(packet_complete) begin
              case (rx_cmd) //switch case to see which command received
                QUERYREP: begin
                      tagisopen   <= 0; // close tag since we're processing data
                      slotcounter <= slotcounter - 15'd1; // function of query rep is to decrement slot counter, so self explanatory. happens after the always block done - nba
                      
                      if (comm_enable & ((slotcounter-15'd1)==0 | ~use_q)) begin // if slot counter finally 0 after this clk edge (or) direct signal to use_q --> check added feature no. 2
                          commstate     <= STATE_TX; // must transmit RN16 after slot counter reaches 0
                         // next is basically command output to tell tag that data being transmitted is rn. bitsrcselect_RNG is the mux control to the transmitter block.
                         bitsrcselect        <= bitsrcselect_RNG;              
                         docrc         <= 0;// since a rn is being sent anyway, crc16 not needed?
                      end else begin
                          rx_en <= 0;  // reset rx, await next command
                      end
                end
                ACK: begin
                      tagisopen <= 0;
                      if (comm_enable && handlematch) begin
                        commstate <= STATE_TX; // send epc.
                        bitsrcselect    <= bitsrcselect_EPC;// whoaaaa now send epc, unless specified to send the tag uid. check added point 3
                        docrc     <= 1;// since we're sending epc
                      end else begin
                      rx_en <= 0;  // reset rx, same principle
                      end
                end
                QUERY: begin
                      tagisopen <= 0;
                      rx_q_reg  <= rx_q; //they're the same, but the reg is it's actual memory
                      // load slot counter
                      slotcounter <= newslotcounter; // new slot counter taken from the received data about Q, line 141. But slot is a random number anyway, just length has to be specified
                      
                      //same code that was used in queryrep, for when slot == 0
                      if (comm_enable & (newslotcounter==0 | ~use_q) & ~crc5invalid) begin // if the newslot counter, which wll be loaded into our slot counter after this clkedge over, turns 0
                        commstate     <= STATE_TX;
                        bitsrcselect        <= bitsrcselect_RNG;
                        docrc         <= 0;
                      end else begin
                        rx_en <= 0;  // reset rx
                      end
                end
                QUERYADJ: begin
                      tagisopen <= 0;
                      rx_q_reg  <= q_adj; // change q value
                      // load slot counter
                      slotcounter <= newslotcounter; // by then slot counter mask will change, so new slot counter will be updated with new length rn
                      
                      //same code that was used in queryrep, for when slot == 0
                      if (comm_enable & (newslotcounter==0 | ~use_q)) begin
                        commstate     <= STATE_TX;
                        bitsrcselect        <= bitsrcselect_RNG;
                        docrc         <= 0;
                      end else begin
                        rx_en <= 0;  // reset rx
                      end
                end
                SELECT: begin
                      tagisopen <= 0;
                      rx_en <= 0;  // reset rx
                end
                NACK: begin
                      tagisopen <= 0;
                      rx_en     <= 0;  // reset rx
                end
                REQRN: begin
                      if (comm_enable && handlematch && ~crc16invalid) begin
                        // First, request_RN opens tag then sets handle, which will be another random number
                        if (!tagisopen) begin
                          storedhandle <= currentrn;
                          tagisopen    <= 1;
                        end
    
                        commstate <= STATE_TX; 
                        bitsrcselect    <= bitsrcselect_RNG;// because need to tx rn
                        docrc     <= 1;
                      end else begin
                        rx_en <= 0;  // reset rx
                      end
                end
                READ: begin
                      if (comm_enable && handlematch && ~crc16invalid) begin
                        if (readwriteptr == 0) readfrommsp <= 0;
                        else                   readfrommsp <= 1;
                        commstate  <= STATE_TX;
                        bitsrcselect     <= bitsrcselect_ADC;
                        docrc      <= 1;
                      end else begin
                        rx_en <= 0;  // reset rx
                      end
                end
                WRITE: begin
                      rx_en <= 0;  // reset rx
                end
                ///
                TRNS: begin
                     tagisopen   <= 0;
                     
                     //turn off calibration bit after 100us. Some parameter pll_dur for no of clk cycles that equate upto 100us
                     if(pllwaitcount >= pll_dur) begin
                         plloff <= 1;
                         pllwaitcount <= 0;
                         rx_en <= 0;
                     end else begin
                         pllwaitcount <= pllwaitcount + 10'd1;
                     end
                end
                SAMPSENS: begin
                     tagisopen   <= 0;
                     //want to send a bit high to the adc to sample data, adc_sample
                     adc_sample  <= 1;
                     rx_en <= 0;
                end
                SENSDATA: begin
                     if (comm_enable && handlematch) begin
                        commstate  <= STATE_TX;
                        bitsrcselect     <= bitsrcselect_ADC;
                        docrc      <= 1;
                     end else begin
                        rx_en <= 0;  // reset rx
                     end
                end
                BFCONST: begin
                     tagisopen  <= 0;
                     rx_en <= 0;
                end
                default begin
                   rx_en <= 0;  // reset rx
                end
              endcase
            end else if(rx_overflow) begin
              rx_en <= 0;
            end else begin
              rx_en <= 1;
              tx_en <= 0;
            end // if packetcomplete
        end else begin
            rx_en <= 0;
        end// sel checking
    end //reset
  end//always
endmodule

