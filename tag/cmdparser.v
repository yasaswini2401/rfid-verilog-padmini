//modifications done




// Command Parser: Parses reader command and signals packet completion.
// Copyright 2010 University of Washington
// License: http://creativecommons.org/licenses/by/3.0/
// 2008 Dan Yeager and Oliver C.

// This module also parses the query tx settings.
// This should be moved to packetparse eventually.
// (possibly along with the cmd parsing)
 
module cmdparser (reset, clk, bitin, bitclk, cmd_out, packet_complete_out, cmd_complete,
                  m, trext, dr, crc5invalid, crc16invalid);
  
  input        reset, clk, bitin, bitclk;
  output       packet_complete_out, cmd_complete;
  ///
  output [11:0] cmd_out;//12 diff commands
  output [1:0] m;
  output       trext, dr;

  reg        packet_complete_out;
  ///
  wire [12:0] cmd_out;
  wire       packet_complete, cmd_complete;
  reg  [7:0] cmd;
  wire [7:0] new_cmd;
  reg  [5:0] count;
  reg  [1:0] m;
  reg        trext, dr; // modulation settings from query
  
  
  //crc5 checking  
  reg crc5reset;
  wire [4:0] crc5out;
  output reg crc5invalid;
  //module crc5(reset, crcinclk, crcbitin, crcout);
  crc5 crc5check(crc5reset, bitclk, bitin, crc5out); 
  
  //crc16 checking
  reg crc16reset;
  wire [16:0] crc16out;
  output reg crc16invalid;
  //module crc16check(reset, crcinclk, crcbitin, crc);
  crc16check  crc16c(crc16reset, bitclk, bitin, crc16out);
   
    

  always @ (posedge bitclk or posedge reset) begin
    if(reset) begin
      count <= 0;
      cmd   <= 0;
      m     <= 0;
      dr    <= 0;
      trext <= 0;
      packet_complete_out <= 0;
      
      //crc5
      crc5reset         <= 1;
      //crc16
      crc16reset        <= 1;
      
      crc5invalid <= 0;
      crc16invalid <= 0;
    
 
    end else begin
    
      cmd   <= new_cmd;
      count <= count + 6'd1;
      packet_complete_out <= packet_complete;  // clear up glitching?

      if(cmd_out[2] && count == 4) dr    <= bitin;
      if(cmd_out[2] && count == 5) m[1]  <= bitin;
      if(cmd_out[2] && count == 6) m[0]  <= bitin;
      if(cmd_out[2] && count == 7) trext <= bitin;
      
      crc5reset <= 0;
      crc16reset <= 0;
      
      //crc checks
      if(cmd_complete) begin
        if(!cmd_out[2]) crc5reset <= 1;
        
        if(!(cmd_out[4] || cmd_out[11] || cmd_out[6] || cmd_out[7] || cmd_out[8])) crc16reset <= 1;
      end
      
      if(packet_complete) begin
        if(cmd_out[2]) begin
            if(crc5out != 5'd0) crc5invalid <= 1;
        end
        
        if((cmd_out[4] || cmd_out[11] || cmd_out[6] || cmd_out[7] || cmd_out[8])) begin
            if(crc16out != 16'h1D0F) crc16invalid <= 1;
        end
      end
      
    end
  end
  

   
  
  assign cmd_complete = (cmd_out > 0);
  
  // we are gating this signal, so update it 1 count early
  //   causing the complete to occur on the last bit pos edge.
  assign packet_complete = ((cmd_out[0] && count >= 3 ) ||  // QueryRep
                            (cmd_out[1] && count >= 17) ||  // Ack
                            (cmd_out[2] && count >= 21) ||  // Query
                            (cmd_out[3] && count >= 8 ) ||  // QueryAdj
                            (cmd_out[4] && count >= 44) ||  // Select
                            (cmd_out[5] && count >= 7 ) ||  // Nack
                            (cmd_out[6] && count >= 39) ||  // ReqRN
                            (cmd_out[7] && count >= 57) ||  // Read, 58 bits assuming 8 bit pointer
                            (cmd_out[8] && count >= 58))||   // Write
                            ///
                            (cmd_out[9] && count >= 13) ||   //trans added command 11011010 plus 6 bits
                            ////
                            (cmd_out[10] && count >= 26)||   //11011111, sample sensor data plus 3 bits plus crc16
                            /////
                            (cmd_out[11] && count >= 43)||  //read sensor data - 8+1+3+32 = 44
                            (cmd_out[12] && count >= 51);
                            
  
  
  assign cmd_out[0] = (count >= 2 && ~cmd[0] && ~cmd[1]);  // QueryRep 00
  assign cmd_out[1] = (count >= 2 && ~cmd[0] &&  cmd[1]);  // Ack 01
  assign cmd_out[2] = (count >= 4 &&  cmd[0] && ~cmd[1] && ~cmd[2] && ~cmd[3]); // query 1000
  assign cmd_out[3] = (count >= 4 &&  cmd[0] && ~cmd[1] && ~cmd[2] &&  cmd[3]); // QueryAdj 1001
  assign cmd_out[4] = (count >= 4 &&  cmd[0] && ~cmd[1] &&  cmd[2] && ~cmd[3]); // Select 1010
  assign cmd_out[5] = (count >= 8 &&  cmd[0] &&  cmd[1] && ~cmd[6] && ~cmd[7]); //Nack 11000000
  assign cmd_out[6] = (count >= 8 &&  cmd[0] &&  cmd[1] && ~cmd[6] &&  cmd[7]); // ReqRN 11000001
  assign cmd_out[7] = (count >= 8 &&  cmd[0] &&  cmd[1] &&  cmd[6] && ~cmd[7] && ~cmd[3]); // Read 11000010
  assign cmd_out[8] = (count >= 8 &&  cmd[0] &&  cmd[1] &&  cmd[6] &&  cmd[7] && ~cmd[3]); // Write 11000011
  ///
  assign cmd_out[9] = (count >= 8 &&  cmd[0] &&  cmd[1] &&  cmd[6] && ~cmd[7] &&  cmd[3]); //added 11011010
  ////
  assign cmd_out[10] = (count >= 8 &&  cmd[0] && cmd[1] && ~cmd[2] &&  cmd[6] &&  cmd[7] && cmd[3]); //11011111
  /////
  assign cmd_out[11] = (count >= 8 &&  cmd[0] && cmd[1] && ~cmd[2] && ~cmd[6] && ~cmd[7] && cmd[3]);//11011000
  assign cmd_out[12] = (count >= 8 &&  cmd[0] && cmd[1] && ~cmd[2] &&  cmd[6] && ~cmd[7] && cmd[3]);//11011110 
  
                 
                 
                            
  assign new_cmd[0] = (count==0) ? bitin : cmd[0];
  assign new_cmd[1] = (count==1) ? bitin : cmd[1];
  assign new_cmd[2] = (count==2 && !cmd_complete) ? bitin : cmd[2];
  assign new_cmd[3] = (count==3 && !cmd_complete) ? bitin : cmd[3];
  assign new_cmd[4] = (count==4 && !cmd_complete) ? bitin : cmd[4];
  assign new_cmd[5] = (count==5 && !cmd_complete) ? bitin : cmd[5];
  assign new_cmd[6] = (count==6 && !cmd_complete) ? bitin : cmd[6];
  assign new_cmd[7] = (count==7 && !cmd_complete) ? bitin : cmd[7];
  

endmodule

  
  
    
