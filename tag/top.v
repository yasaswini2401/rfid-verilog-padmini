///modified code


// Top level which connects all the top-level functional blocks.
// Copyright 2010 University of Washington
// License: http://creativecommons.org/licenses/by/3.0/
// 2008 Dan Yeager

// The controller chooses if and what packet is sent.

// RX converts RFID protocol into a serial data stream
//    and provides TRCAL, the tx clock divider calibration
//    and the lsb of the counter as a random number stream.

// CMDPARSE and PACKETPARSE decode the serial bit stream
//    and help the controller make decisions.

// TX converts a serial bit stream to RFID protocol.
//    It is wrapped in a SEQUENCER which provides the proper
//    clock to TX and sequences the preamble, DATA and crc in time.

// The controller connects one of 4 DATA sources to the sequencer.
//    Options are RNG (random number), EPC (ID), READ (response to READ packet)
//    and WRITE (response to WRITE packet).

    module top(reset, clk, demodin, modout, // regular IO
           adc_sample_ctl, adc_sample_clk, adc_sample_datain,
           writedataout, writedataclk, 
           use_q, comm_enable, tx_enable,
           debug_clk, debug_out,
           rx_cmd,
           ///transmit clock
           pll_enable, freq_channel, rforbase,
           ////from sample data
           adc_sample, senscode,
           /////
           morb_trans, sensor_time_stamp, bf_dur,      
           crc5invalid, crc16invalid, bitout,
           calibration_control,
           //select
           sel_target, sel_action, sel_ptr, mask,
           sl_flag);

  // Regular IO
  // Oscillator input, master reset, demodulator input
  input  reset, clk, demodin;

  // Modulator output
  output modout;

  // Functionality control, removed use_uid
  input use_q, comm_enable;

  // EPC ID source, uid removed

  // ADC connections
  input  adc_sample_datain;
  output adc_sample_clk, adc_sample_ctl;

  
  output [15:0] writedataout; 
  output writedataclk;

  // Debugging IO
  input  debug_clk;
  output debug_out;
  
/**********Additional commands**********/
  
  /// trns: transmit clock command, all connected to packetparse module
  output wire pll_enable; /// final control for pll
  output wire [3:0] freq_channel;
  output wire rforbase;
  //// sampsens: wilo router asks tag to sample data
  output wire [2:0] senscode;
  output wire adc_sample;
  ///// sensdata: wilo router asks for sensor data
  output wire morb_trans; //main or backscatter transmitter
  output wire [7:0] sensor_time_stamp;
  /// bfconst: ask tag to backscatter at constant frequency
  //output wire freq
  output wire [7:0] bf_dur;
  
  //for select: sel_target, sel_action, sel_ptr, mask - not using mask length since redundant
  output wire [2:0] sel_target;
  output wire [2:0] sel_action;
  output wire [7:0] sel_ptr;
  output wire [15:0] mask;
  input sl_flag;
  
  //crc5 and crc16 checks
  output crc5invalid, crc16invalid;
  
  //clock recovery circuit signals
  output wire calibration_control;
  output bitout;
  //tx_enable turns on 100 clk cycles (of 2MHz clk) before tx starts 
  output wire tx_enable; // is the same as txsetupdone, so assign it
  
  
  
/********************Module connections********************/
   
  // CONTROLLER module connections
  wire rx_en, tx_en, docrc;
  wire [15:0] currentrn;     // current rn
  wire [15:0] currenthandle; // current handle
  wire plloff; // driven to turn off pll, for a specific amount of clock cycles

  // RX module connections
  wire rx_reset, rxtop_reset, bitclk;
  wire bitout;
  wire rx_overflow;

  // PACKET PARSE module connections
  wire       handlematch;
  wire [1:0] readwritebank;//use for read, write, select
  wire [7:0] readwriteptr; //read write
  wire [7:0] readwords; //read
  wire [15:0] writedataout;
  wire writedataclk; //write
  wire [3:0] rx_q;//query
  wire [1:0] sel;
  wire [2:0] rx_updn;//queryadj
   

  // CMDPARSE module connections
  wire packet_complete, cmd_complete;
  output wire [12:0] rx_cmd;
      //crc5, crc16 checks
      wire crc5invalid, crc16invalid;

  // TX module connections
  wire tx_reset, txsetupdone, tx_done;

  // TX settings module wires
  wire dr_in, dr_out;
  wire trext_in, trext_out;
  wire [1:0] m_in, m_out;
  wire [9:0] trcal_in, trcal_out;

  // Signal to tx settings module to store TR modulation settings.
  parameter QUERY = 13'b000000000100;
  wire query_complete;
  assign query_complete = packet_complete && (rx_cmd==QUERY);

  // RNG connections
  wire rngbitin, rngbitinclk;
  // Signal to RNG to clock in new bits for query, queryadj, reqrn
  assign rngbitinclk = bitclk & (rx_cmd[2] | rx_cmd[3] | (rx_cmd[6] & handlematch));

  // TX module connections
  wire txbitclk, txbitsrc, txdatadone;
  
  /*************************************************************/  
  //tx_enable - output that is the enable signal for transmission  
  assign tx_enable = txsetupdone;
  
  // RX and TX module reset signals
  assign tx_reset    = reset | !tx_en; // transmission is not allowed if crc5 is invalid
  assign rx_reset    = reset | !rx_en;
  assign rxtop_reset = reset | !rx_en;
  
  /// this is for trns command
  //mux control for pll on and off
  wire pllenab;
  assign pll_enable = (pllenab) & (~plloff);

  // mux control for transmit data source
  wire [1:0] bitsrcselect;
  parameter BITSRC_RNG  = 2'd0;
  parameter BITSRC_EPC  = 2'd1;
  parameter BITSRC_READ = 2'd2;
  
//bitsrc all msb first


  // mux the bit source for the tx module, uidbitsrc removed, so 2:0 instead of 3:0
  wire [2:0] bitsrc;
  wire rngbitsrc, epcbitsrc, readbitsrc;
  assign bitsrc[0] = rngbitsrc;
  assign bitsrc[1] = epcbitsrc;
  assign bitsrc[2] = readbitsrc;
  assign txbitsrc  = bitsrc[bitsrcselect];

  // mux control for data source done flag, uidddatadone removed
  wire [2:0] datadone;
  wire rngdatadone, epcdatadone, readdatadone;
  assign datadone[0] = rngdatadone;
  assign datadone[1] = epcdatadone;
  assign datadone[2] = readdatadone;
 
  assign txdatadone  = datadone[bitsrcselect];

  // mux control for tx data clock, uidbitclk removed
  wire   rngbitclk, epcbitclk, readbitclk;
  assign rngbitclk  = (bitsrcselect == BITSRC_RNG ) ? txbitclk : 1'b0;
  assign epcbitclk  = (bitsrcselect == BITSRC_EPC ) ? txbitclk : 1'b0;
  assign readbitclk = (bitsrcselect == BITSRC_READ) ? txbitclk : 1'b0;
  

/*************Connections we don't need- adc or msp340************/
  // MUX connection from READ to MSP or ADC
  wire readfrommsp;
  wire readfromadc = !readfrommsp;
  wire read_sample_ctl, read_sample_clk, read_sample_datain;

  // ADC connections
  assign adc_sample_ctl     = read_sample_ctl    & readfromadc;
  assign adc_sample_clk     = read_sample_clk    & readfromadc;

  // MSP430 connections, removed 
  
  assign read_sample_datain = adc_sample_datain;
/*******************************************************************/


  // Serial debug interface for viewing registers:
  reg [3:0] debug_address;
  reg debug_out;
  always @ (posedge debug_clk or posedge reset) begin
    if(reset) begin
      debug_address <= 4'd0;
    end else begin
      //debug_address <= debug_address + 4'd1;
      debug_address <= 4'd11;
    end
  end
  always @ (debug_clk) begin // always @ (debug_address) begin // --> there initially
  case(debug_address)
    0:  debug_out = packet_complete;
    1:  debug_out = cmd_complete;
    2:  debug_out = handlematch;
    3:  debug_out = docrc;
    4:  debug_out = rx_en;
    5:  debug_out = tx_en;
    6:  debug_out = bitout;
    7:  debug_out = bitclk;
    8:  debug_out = rngbitin;
    9:  debug_out = rx_overflow;
    10: debug_out = tx_done;
    11: debug_out = txsetupdone;
    12: debug_out = 1'b0;
    13: debug_out = 1'b1;
    14: debug_out = 1'b0;
    15: debug_out = 1'b1;
    default: debug_out = 1'b0;
  endcase
  end
  
  //for cdr circuit
  wire preamble_indicator;
  
  reg ext_packetcomplete;
  assign calibration_control = ~(ext_packetcomplete);
    
  always@(posedge packet_complete or posedge preamble_indicator) begin
    if(preamble_indicator == 0) begin
        ext_packetcomplete <= 1;
    end else ext_packetcomplete <= 0;
  end
  

/******************** MODULES! :)********************/

  controller U_CTL (reset, clk, rx_overflow, rx_cmd, currentrn, currenthandle,
                    packet_complete, txsetupdone, tx_done, 
                    rx_en, tx_en, docrc, handlematch,
                    bitsrcselect, readfrommsp, readwriteptr, rx_q, rx_updn,
                    use_q, comm_enable,
                    ///
                    plloff,
                    ////
                    adc_sample, 
                    crc5invalid, crc16invalid, sel, sl_flag);

  txsettings U_SET (reset, trcal_in,  m_in,  dr_in,  trext_in, query_complete,
                           trcal_out, m_out, dr_out, trext_out);

  rx        U_RX  (rx_reset, clk, demodin, bitout, bitclk, rx_overflow, trcal_in, rngbitin, preamble_indicator);
  
  cmdparser U_CMD (rxtop_reset, clk, bitout, bitclk, rx_cmd, packet_complete, cmd_complete,
                   m_in, trext_in, dr_in, crc5invalid, crc16invalid);

  packetparse U_PRSE (rx_reset, bitout, bitclk, rx_cmd, rx_q, sel, rx_updn,
                      currenthandle, currentrn, handlematch,
                      readwritebank, readwriteptr, readwords,
                      writedataout, writedataclk,
                      /// trns
                      pllenab, freq_channel,rforbase,
                      //// sampsens
                      senscode,
                      ///// sensdata
                      morb_trans, sensor_time_stamp,
                      // bfconst 
                      bf_dur,
                      sel_target, sel_action, sel_ptr, mask); // not using truncate 

  rng       U_RNG  (tx_reset, reset, rngbitin, rngbitinclk, rngbitclk, rngbitsrc, rngdatadone, currentrn);
  
  
    epc_linemem       U_EPC  (tx_reset, epcbitclk, epcbitsrc, epcdatadone);
  
  read      U_READ (tx_reset, readbitclk, readbitsrc, readdatadone, 
                    read_sample_ctl, read_sample_clk, read_sample_datain, 
                    currenthandle);
                    
                    
  //uid removed
                    
//module sequencer (reset, rtcal_expired, oscclk, m, dr, docrc, trext, 
                  //trcal, databitsrc, datadone, dataclk, modout, txsetupdone, txdone);
  sequencer U_SEQ (tx_reset, rx_overflow, clk, m_out, dr_out, docrc, trext_out, 
                   trcal_out, txbitsrc, txdatadone, txbitclk, modout, txsetupdone, tx_done);


 //adding extra module epc, for memory purpose
  //epc_memorymodule(clk, reset, curr_sl_flag, curr_inven_flag)
                   
endmodule
