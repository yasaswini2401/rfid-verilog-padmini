`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/05/2023 03:42:34 PM
// Design Name: 
// Module Name: mem
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module mem (
    input wire clk,factory_reset,reset,packetcomplete,
    input wire [12:0] rx_cmd,
    input wire [2:0] sel_target,
    input wire [2:0] sel_action,
    input wire [7:0] sel_ptr,
    input wire [7:0] sel_masklen,
    input wire [15: 0] mask,  
    input wire [1:0] readwritebank,
    input wire [7:0] readwriteptr, 
    input wire [7:0] readwords,
    input wire [15:0]writedataout,
    input wire ADC_data_ready,
    input wire [10:0]ADC_data,
    input wire [2:0]sensor_code, // 3-bit flag to indicate the 3 sensors
    input wire [15:0]data_in,  //data from memory
    input wire word_tx_done,
    output reg [15:0] data_out, //data is given to the memory
    output reg PC_B,WE,SE,
    output reg [5:0]mem_address,   //this will enable WL
    output reg [1:0]mem_sel,
    output reg [15:0]sensor_data,
    output reg [15:0]EPC_dataout, // EPC data sent out to the transmitter from the interface
    output reg adc_done,
    output reg sl_flag,inven_flag,
    output reg [1:0]session
);
reg [5:0]counter_EPC,counter_s1,counter_s2;
reg [15:0] StoredCRC, StoredPC, Code1; // the first three 16-bit words of EPC
wire [15:0] CRC,PC; 
reg curr_sl_flag,curr_inven_flag,read_state,write_state,current_cmd,RorW,adc_flag;
reg [5:0]temp;
reg [7:0]adc_temp_data;  
// commands 
parameter CMD_ACK = 3'd0;
parameter CMD_EPC_READ = 3'd1;
parameter CMD_SENSOR_READ = 3'd2;
parameter CMD_EPC_WRITE = 3'd4;
//RorW
parameter RorW_INITIAL = 6'd0;
parameter EPC_READ = 6'd1;
parameter SENSOR1_READ = 6'd2;
parameter SENSOR2_READ = 6'd4;
parameter EPC_WRITE = 6'd8;
parameter SENSOR1_WRITE = 6'd16;
parameter SENSOR2_WRITE = 6'd32;
//ADC
parameter ADC_DATA_READY_FLAG = 1'd1;
// read and write states
parameter STATE_INITIAL = 8'd0;
parameter STATE_RESET = 8'd1;
parameter STATE_MEM_SEL = 8'd2;
parameter STATE_PC_B = 8'd4;
parameter STATE_MEM_ADDRESS = 8'd8;
parameter STATE_WE = 8'd16;
parameter STATE_SE = 8'd32;
parameter STATE_DATAIN = 8'd64;
parameter STATE_DATAOUT = 8'd128;
always@(posedge clk or posedge reset or posedge factory_reset)begin
    if(factory_reset)begin
        counter_EPC <= 0;
        counter_s1 <= 0;
        counter_s2 <= 0;
        curr_inven_flag <=1;
        curr_sl_flag <=1;
    end else if(reset)begin
        data_out <= 0;
        sl_flag <= 1;            //need to change
        inven_flag <= 1;         //need to change
        session <= 0;
        PC_B <= 1;
        SE <= 0;
        WE <= 0;
        adc_done<=0;
        read_state <= STATE_INITIAL;
        write_state <= STATE_INITIAL;
        RorW <= RorW_INITIAL;
    end else begin
        
        if(counter_EPC == 3)begin
           Code1 = writedataout ;     
        end
        
        if(packetcomplete)begin
            if(rx_cmd[4])begin//if command is select
                if(readwritebank == 2'b01)begin // if membank is 01
                    case(sel_target)
                      3'b000: session = 2'b00;
                      3'b001: session = 2'b01;
                      3'b010: session = 2'b10;
                      3'b011: session = 2'b11;
                      3'b100: sl_flag = 1'b1;
                    endcase
                
                    if(Code1 == mask)begin // if tag is matching
                        case(sel_action)
                          3'b000:if(sel_target < 3'b100)begin inven_flag = 1'b1; end else if(sel_target == 3'b100)begin sl_flag = 1'b1; end
                          3'b001:if(sel_target < 3'b100)begin inven_flag = 1'b1; end else if(sel_target == 3'b100)begin sl_flag = 1'b1; end
                          3'b011:if(sel_target < 3'b100)begin inven_flag = !curr_inven_flag; end else if(sel_target == 3'b100)begin sl_flag = !curr_sl_flag; end
                          3'b100:if(sel_target < 3'b100)begin inven_flag = 1'b0; end else if(sel_target == 3'b100)begin sl_flag = 1'b0; end
                          3'b001:if(sel_target < 3'b100)begin inven_flag = 1'b0; end else if(sel_target == 3'b100)begin sl_flag = 1'b0; end
                        endcase
                    end else begin
                        case(sel_action)
                          3'b000:if(sel_target < 3'b100)begin inven_flag = 1'b0; end else if(sel_target == 3'b100)begin sl_flag = 1'b0; end
                          3'b010:if(sel_target < 3'b100)begin inven_flag = 1'b0; end else if(sel_target == 3'b100)begin sl_flag = 1'b0; end
                          3'b100:if(sel_target < 3'b100)begin inven_flag = 1'b1; end else if(sel_target == 3'b100)begin sl_flag = 1'b1; end
                          3'b110:if(sel_target < 3'b100)begin inven_flag = 1'b1; end else if(sel_target == 3'b100)begin sl_flag = 1'b1; end
                          3'b111:if(sel_target < 3'b100)begin inven_flag = !curr_inven_flag; end else if(sel_target == 3'b100)begin sl_flag = !curr_sl_flag; end
                        endcase
                    end
                   end
                   curr_inven_flag = inven_flag;
                   curr_sl_flag = sl_flag;
                end //membank
            end//select command
        
        if(rx_cmd[1])begin   //acknowledge command
            current_cmd = CMD_ACK;
        end else if(rx_cmd[7])begin  // read command (epc)
            current_cmd = CMD_EPC_READ;
        end else if(rx_cmd[11])begin
            current_cmd = CMD_SENSOR_READ;  // sensor read command
        end else if(rx_cmd[8])begin
            current_cmd = CMD_EPC_WRITE;   //write command (epc)
        end
            
        if(current_cmd == CMD_ACK)begin     
            if(read_state == STATE_INITIAL)begin
                  mem_sel <= 2'd1;
                  read_state <= STATE_MEM_SEL;
            end else if(read_state == STATE_MEM_SEL)begin
                  PC_B <= 0;
                  read_state <= STATE_PC_B;
            end else if(read_state == STATE_PC_B)begin
                  mem_address <= counter_EPC-1;                //need to check
                  read_state <= STATE_MEM_ADDRESS;
            end else if(read_state == STATE_MEM_ADDRESS)begin
                  SE <= 1;
                  read_state <= STATE_SE;
            end else if(read_state == STATE_SE)begin
                if(word_tx_done)begin
                  EPC_dataout <= data_in;
                  counter_EPC <= counter_EPC -1;
                  read_state <= STATE_DATAIN;
                  end
            end else if(read_state == STATE_DATAIN)begin
                if(counter_EPC!=0)begin
                 read_state <= STATE_INITIAL;
                end
            end
        end
        
        if(current_cmd == CMD_EPC_READ)begin
            if(packetcomplete)begin 
                if(readwritebank == 2'b01)begin
                    RorW <= EPC_READ;
                    temp <= readwriteptr+readwords-1;
                end
            end
        end else if(current_cmd == CMD_SENSOR_READ)begin
            if(sensor_code == 3'd1)begin
                RorW <= SENSOR1_READ;
            end
            else if(sensor_code == 3'd2)begin
                RorW <= SENSOR2_READ;
            end
        end else if(current_cmd == CMD_EPC_WRITE)begin
            if(packetcomplete)begin 
                if(readwritebank == 2'b01)begin
                    RorW <= EPC_WRITE;
                end
            end
        end
        
        
        if(RorW == EPC_READ)begin     
            if(read_state == STATE_INITIAL)begin
                  mem_sel <= 2'd1;
                  read_state <= STATE_MEM_SEL;
            end else if(read_state == STATE_MEM_SEL)begin
                  PC_B <= 0;
                  read_state <= STATE_PC_B;
            end else if(read_state == STATE_PC_B)begin
                  mem_address <= temp;                   //need to modify, wrong code
                  read_state <= STATE_MEM_ADDRESS;
            end else if(read_state == STATE_MEM_ADDRESS)begin
                  SE <= 1;
                  read_state <= STATE_SE;
            end else if(read_state == STATE_SE)begin
                if(word_tx_done)begin
                  EPC_dataout <= data_in;
                  temp <= temp -1;
                  read_state <= STATE_DATAIN;
                  end
            end else if(read_state == STATE_DATAIN)begin
                if(temp!=readwriteptr-1)begin
                 read_state <= STATE_INITIAL;
                end
            end
        end
        
        if(RorW == SENSOR1_READ)begin     
            if(read_state == STATE_INITIAL)begin
                  mem_sel <= 2'd2;
                  read_state <= STATE_MEM_SEL;
            end else if(read_state == STATE_MEM_SEL)begin
                  PC_B <= 0;
                  read_state <= STATE_PC_B;
            end else if(read_state == STATE_PC_B)begin
                  mem_address <= counter_s1-1;                   
                  read_state <= STATE_MEM_ADDRESS;
            end else if(read_state == STATE_MEM_ADDRESS)begin
                  SE <= 1;
                  read_state <= STATE_SE;
            end else if(read_state == STATE_SE)begin
                 if(word_tx_done)begin
                  sensor_data <= data_in;
                  counter_s1 <= counter_s1-1;
                  read_state <= STATE_DATAIN;
                  end
            end else if(read_state == STATE_DATAIN)begin
                if(counter_s1!=0)begin
                 read_state <= STATE_INITIAL;
                end
            end
        end
        
        if(RorW == SENSOR2_READ)begin     
            if(read_state == STATE_INITIAL)begin
                  mem_sel <= 2'd3;
                  read_state <= STATE_MEM_SEL;
            end else if(read_state == STATE_MEM_SEL)begin
                  PC_B <= 0;
                  read_state <= STATE_PC_B;
            end else if(read_state == STATE_PC_B)begin
                  mem_address <= counter_s2-1;                  
                  read_state <= STATE_MEM_ADDRESS;
            end else if(read_state == STATE_MEM_ADDRESS)begin
                  SE <= 1;
                  read_state <= STATE_SE;
            end else if(read_state == STATE_SE)begin
                 if(word_tx_done)begin
                  sensor_data <= data_in;
                  counter_s2 <= counter_s2-1;
                  read_state <= STATE_DATAIN;
                  end
            end else if(read_state == STATE_DATAIN)begin
                if(counter_s2!=0)begin
                 read_state <= STATE_INITIAL;
                end
            end
        end
             
        if(ADC_data_ready)begin //have to wait for one clock cycle
            adc_flag <= ADC_DATA_READY_FLAG;
        end 
       
       if(adc_flag == ADC_DATA_READY_FLAG)begin 
          if(ADC_data[10:8] == 3'b001)begin  //sensor 1
            RorW <= SENSOR1_WRITE;
           end else if(ADC_data[10:8] == 3'b010)begin  //sensor 2
            RorW <= SENSOR2_WRITE;
           end
          adc_temp_data <= ADC_data[7:0];
        end
        
        if(RorW == SENSOR1_WRITE)begin 
            if(write_state == STATE_INITIAL)begin
                  mem_sel <= 2'd2;
                  read_state <= STATE_MEM_SEL;    
            end else if(write_state == STATE_MEM_SEL)begin
                  PC_B <= 0;
                  write_state <= STATE_PC_B;
            end else if(write_state == STATE_PC_B)begin
                  mem_address <= counter_s1;                  
                  write_state <= STATE_MEM_ADDRESS;
            end else if(write_state == STATE_MEM_ADDRESS)begin
                  WE <= 1;
                  write_state <= STATE_WE;
            end else if(write_state == STATE_WE)begin
                  data_out <= adc_temp_data;
                  counter_s1 <= counter_s1+1;
                  write_state <= STATE_DATAOUT;
            end else if(write_state == STATE_DATAOUT)begin
                  adc_done <= 1;
                  write_state <= STATE_RESET;
            end else if(write_state == STATE_RESET)begin
                  adc_done <= 0;
                  adc_flag <= 0;
                  write_state <= STATE_INITIAL;
                  RorW <= RorW_INITIAL;
                  end
        end
        
        if(RorW == SENSOR2_WRITE)begin
            if(write_state == STATE_INITIAL)begin
                      mem_sel <= 2'd2;
                      read_state <= STATE_MEM_SEL;          
            end else if(write_state == STATE_MEM_SEL)begin
                  PC_B <= 0;
                  write_state <= STATE_PC_B;
            end else if(write_state == STATE_PC_B)begin
                  mem_address <= counter_s2;                  
                  write_state <= STATE_MEM_ADDRESS;
            end else if(write_state == STATE_MEM_ADDRESS)begin
                  WE <= 1;
                  write_state <= STATE_WE;
            end else if(write_state == STATE_WE)begin
                  data_out <= adc_temp_data;
                  counter_s2 <= counter_s2+1;
                  write_state <= STATE_DATAOUT;
            end else if(write_state == STATE_DATAOUT)begin
                  adc_done <= 1;
                  write_state <= STATE_RESET;
            end else if(write_state == STATE_RESET)begin
                  adc_done <= 0;
                  adc_flag <= 0;
                  write_state <= STATE_INITIAL;
                  RorW <= RorW_INITIAL;
                  end
            end
        end
        
       if(RorW == EPC_WRITE)begin
            if(read_state == STATE_INITIAL)begin
                  mem_sel <= 2'd1;
                  read_state <= STATE_MEM_SEL;    
            end else if(write_state == STATE_MEM_SEL)begin
                  PC_B <= 0;
                  write_state <= STATE_PC_B;
            end else if(write_state == STATE_PC_B)begin
                  mem_address <= readwriteptr;                  
                  write_state <= STATE_MEM_ADDRESS;
            end else if(write_state == STATE_MEM_ADDRESS)begin
                  WE <= 1;
                  write_state <= STATE_WE;
            end else if(write_state == STATE_WE)begin
                  data_out <= writedataout;
                  counter_EPC <= counter_EPC+1;
                  write_state <= STATE_DATAOUT;
            end else if(write_state == STATE_DATAOUT)begin
                  write_state <= STATE_INITIAL;
                  RorW <= RorW_INITIAL;
            end
        end 
end//always
endmodule
