//final as of 20-03-2023


`timescale 1ns / 1ps

module mem(
    input wire clk,factory_reset,reset,packetcomplete,
    input wire [12:0] rx_cmd,
    input wire [2:0] sel_target,
    input wire [2:0] sel_action,
    input wire [7:0] sel_ptr,
    input wire [7:0] sel_masklen,
    input wire [15:0] mask,  
    input wire [1:0] readwritebank,
    input wire [7:0] readwriteptr, 
    input wire [7:0] readwords,
    input wire [15:0]EPC_data_in,
    input wire ADC_data_ready,
    input wire EPC_data_ready,     
    input wire [7:0]ADC_data,
    input wire [2:0]sensor_code, // 3-bit flag to indicate the 3 sensors
    input wire [15:0]mem_read_in,  //data from memory
    input wire [7:0]sensor_time_stamp,   
    input wire data_clk,

    output reg [15:0] mem_data_out, //data is given to the memory
    output reg PC_B,WE,SE,
    output reg [5:0]mem_address,   //this will enable WL
    output reg [2:0]mem_sel,
    output reg tx_bit_src,
    output reg mem_done,
    output reg sl_flag,inven_flag,
    output reg [1:0]session,RorW,
    output reg tx_data_done
);

reg [5:0]counter_EPC,counter_s1,counter_s2;
reg [15:0] StoredCRC, StoredPC, Code1,tx_out; // the first three 16-bit words of EPC 
reg curr_sl_flag,curr_inven_flag,adc_flag;
reg [3:0]current_cmd;
reg [3:0]read_state,write_state;
reg [5:0]temp,Read_or_Write;
reg [15:0]adc_temp_data,bit_shift_reg;
reg next_word,words_done;
reg [3:0] bit_counter;  

// commands
parameter CMD_RESET = 4'd0;
parameter CMD_ACK = 4'd1;
parameter CMD_EPC_READ = 4'd2;
parameter CMD_SENSOR_READ = 4'd4;
parameter CMD_EPC_WRITE = 4'd8;

//
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
parameter STATE_INITIAL = 4'd1;
parameter STATE_RESET = 4'd2;
parameter STATE_1 = 4'd4;
parameter STATE_2 = 4'd8;

always@(posedge data_clk or posedge reset)begin
    if(reset)begin
        bit_counter = 4'd0;
        words_done = 1'd0;   
        mem_data_out = 16'd0;
        sl_flag = 1'd1;            
        inven_flag = 1'd1;         
        session = 2'd0;
        PC_B = 1'd1;
        SE = 1'd0;
        WE = 1'd0;
        mem_done =1'd0;
        read_state = STATE_INITIAL;
        write_state = STATE_INITIAL;
        Read_or_Write = RorW_INITIAL;
        mem_sel = 3'd0;
        mem_address = 6'd0;
        tx_data_done = 1'b0;
        RorW = 2'd0;    
    end else begin
    
        if(bit_counter == 4'd2 || (packetcomplete))begin
            next_word = 1'd1;
        end else begin
            next_word = 1'd0;
        end 
        if(bit_counter == 4'd0)begin
            bit_shift_reg = tx_out;
        end else begin
             bit_shift_reg =  bit_shift_reg;
        end
        tx_bit_src = bit_shift_reg[bit_counter];
        
        if((words_done ==4'd1) & (bit_counter == 4'd15))begin
            tx_data_done = 1'd1;
            next_word = 1'd0;
        end else begin
            tx_data_done = 1'd0;
            next_word = 1'd1;
        end
        bit_counter = bit_counter +4'd1;     
      end
end

always@(posedge clk)begin
    if(factory_reset)begin
        counter_EPC = 6'd0;
        counter_s1 = 6'd0;
        counter_s2 = 6'd0;
        curr_inven_flag =1'd1;
        curr_sl_flag =1'd1;
        sl_flag = 1'd1;  
    end else begin
        if(counter_EPC == 6'd3)begin
           Code1 = EPC_data_in ;
        end else begin
           Code1 = 16'd0; 
        end
        
        if(packetcomplete)begin
            if(rx_cmd[4])begin//if command is select
                if(readwritebank == 2'b01)begin // if membank is 01
                    case(sel_target)
                      3'b000: session = 2'b00;
                      3'b001: session = 2'b01;
                      3'b010: session = 2'b10;
                      3'b011: session = 2'b11;
                    endcase
                
                    if(mask == Code1)begin // if tag is matching
                        case(sel_action)
                          3'b000:if(sel_target < 3'b100)begin inven_flag = 1'b1; end else if(sel_target == 3'b100)begin sl_flag = 1'b1; end
                          3'b001:if(sel_target < 3'b100)begin inven_flag = 1'b1; end else if(sel_target == 3'b100)begin sl_flag = 1'b1; end
                          3'b011:if(sel_target < 3'b100)begin inven_flag = !curr_inven_flag; end else if(sel_target == 3'b100)begin sl_flag = !curr_sl_flag; end
                          3'b100:if(sel_target < 3'b100)begin inven_flag = 1'b0; end else if(sel_target == 3'b100)begin sl_flag = 1'b0; end
                          3'b101:if(sel_target < 3'b100)begin inven_flag = 1'b0; end else if(sel_target == 3'b100)begin sl_flag = 1'b0; end
                        endcase
                    end else begin //not matching
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
                end //membank //select command
            end else begin
                curr_inven_flag = inven_flag;
                curr_sl_flag = sl_flag;
         end
        
        if(rx_cmd[1])begin   //acknowledge command
            current_cmd = CMD_ACK;
        end else if(rx_cmd[7])begin  // read command (epc)
            current_cmd = CMD_EPC_READ;
        end else if(rx_cmd[11])begin
            current_cmd = CMD_SENSOR_READ;  // sensor read command
        end else if(rx_cmd[8])begin
            current_cmd = CMD_EPC_WRITE;   //write command (epc)
        end else begin
            current_cmd = CMD_RESET;
        end
        
        if(current_cmd == CMD_EPC_READ)begin
            if(packetcomplete)begin 
                if(readwritebank == 2'b01)begin
                    Read_or_Write = EPC_READ;
                    temp = readwriteptr+readwords-8'd1;
                end
            end
        end else if(current_cmd == CMD_SENSOR_READ)begin
            if(sensor_code == 3'd1)begin
                Read_or_Write = SENSOR1_READ;
            end
            else if(sensor_code == 3'd2)begin
                Read_or_Write = SENSOR2_READ;
            end
        end else if(current_cmd == CMD_EPC_WRITE)begin
            if(EPC_data_ready)begin 
               if(readwritebank == 2'b01)begin
                   Read_or_Write = EPC_WRITE; 
               end
            end
        end else begin
            Read_or_Write = RorW_INITIAL;
        end
        
        if(ADC_data_ready)begin //have to wait for one clock cycle
            adc_flag = ADC_DATA_READY_FLAG;
        end else begin
            adc_flag = 1'd0;
        end
       
       if(adc_flag == ADC_DATA_READY_FLAG)begin 
          if(sensor_code == 3'b001)begin  //sensor 1
            Read_or_Write = SENSOR1_WRITE;
          end else if(sensor_code == 3'b010)begin  //sensor 2
              Read_or_Write = SENSOR2_WRITE;
          end else begin
               Read_or_Write = RorW_INITIAL;
          end
          adc_temp_data = {sensor_time_stamp,ADC_data};
       end else begin
           Read_or_Write = RorW_INITIAL;
       end
        
       if(current_cmd == CMD_ACK)begin     
           if(read_state == STATE_INITIAL)begin
               if(next_word)begin
                 mem_sel = 3'd1;
                 RorW = 2'b01;
                 mem_address = counter_EPC-6'd1; 
                 PC_B = 1'd0;          
                 read_state = STATE_1;
                end
           end else if(read_state == STATE_1)begin
                 PC_B = 1'd1;
                 SE = 1'd1;
                 tx_out = mem_read_in;
                 read_state = STATE_2;
           end else if(read_state == STATE_2)begin
                 counter_EPC = counter_EPC -6'd1;               
                 read_state = STATE_RESET;
           end else begin
                if(counter_EPC!=6'd0)begin
                 read_state = STATE_INITIAL;
                 words_done = 1'd0;   
                end else begin
                 words_done = 1'd1;
                end
                SE = 1'd0;
                RorW = 2'd0;
           end
        end else if(Read_or_Write == SENSOR1_WRITE)begin 
            if(write_state == STATE_INITIAL)begin
                  mem_sel = 3'd2;
                  RorW = 2'b10;
                  PC_B = 1'd0;
                  mem_address = counter_s1;
                  write_state = STATE_1;    
            end else if(write_state == STATE_1)begin
                  PC_B = 1'd1;
                  mem_data_out = adc_temp_data;
                  WE = 1'd1;
                  write_state = STATE_2;
            end else if(write_state == STATE_2)begin
                  counter_s1 = counter_s1+6'd1;
                  mem_done = 1'd1;
                  write_state = STATE_RESET;
            end else begin
                  RorW = 2'd0;
                  WE = 1'd0;
                  adc_flag = 1'd0;
                  write_state = STATE_INITIAL;
                  Read_or_Write = RorW_INITIAL;
                  end
        end else if(Read_or_Write == SENSOR2_WRITE)begin
            if(write_state == STATE_INITIAL)begin
                  mem_sel = 3'd4;
                  RorW = 2'b10;
                  PC_B = 1'd0;
                  mem_address = counter_s2;  
                  write_state = STATE_1;          
            end else if(write_state == STATE_1)begin
                  PC_B = 1'd1;
                  mem_data_out = adc_temp_data;
                  WE = 1'd1;
                  write_state = STATE_2;
            end else if(write_state == STATE_2)begin    
                  counter_s2 = counter_s2+6'd1;
                  mem_done = 1;
                  write_state = STATE_RESET;
            end else begin
                  WE = 1'd0;
                  RorW = 2'd0;
                  adc_flag = 1'd0;
                  write_state = STATE_INITIAL;
                  Read_or_Write = RorW_INITIAL;
                  end
         end else if(Read_or_Write == EPC_WRITE)begin
            if(write_state == STATE_INITIAL)begin
                  mem_sel = 3'd1;
                  RorW = 2'b10;
                  PC_B = 1'd0;
                  mem_address = readwriteptr; 
                  write_state = STATE_1;    
            end else if(write_state == STATE_1)begin
                  PC_B = 1'd1;
                  mem_data_out = EPC_data_in;
                  WE = 1'd1;
                  write_state = STATE_2;
            end else if(write_state == STATE_2)begin
                  counter_EPC = counter_EPC+6'd1;
                  write_state = STATE_RESET;
            end else begin
                  WE = 1'd0;
                  RorW = 2'd0;
                  write_state = STATE_INITIAL;
                  Read_or_Write = RorW_INITIAL;
            end
        end else if(Read_or_Write == EPC_READ)begin     
            if(read_state == STATE_INITIAL)begin
                if(next_word)begin
                  mem_sel = 3'd1;
                  RorW = 2'b01;
                  mem_address = temp; 
                  PC_B = 1'd0;
                  read_state = STATE_1;
                 end
            end else if(read_state == STATE_1)begin
                  PC_B = 1'd1;
                  SE = 1'd1;
                  tx_out = mem_read_in;
                  read_state = STATE_2;
            end else if(read_state == STATE_2)begin
                   temp = temp -6'd1;                  
                  read_state = STATE_RESET;
            end else begin
                if(temp!=readwriteptr-8'd1)begin
                 read_state = STATE_INITIAL;
                 words_done = 1'd0;
                end else begin
                    words_done = 1'd1;
                end
                RorW = 2'd0;
                SE = 1'd0;
            end
        end else if(Read_or_Write == SENSOR1_READ)begin     
            if(read_state == STATE_INITIAL)begin
                if(next_word)begin
                  mem_sel = 3'd2;
                  RorW = 2'b01;
                  mem_address = counter_s1-6'd1;
                  PC_B = 1'd0;
                  read_state = STATE_1;
                 end
            end else if(read_state == STATE_1)begin
                  PC_B = 1'd1;
                  SE = 1'd1;
                  tx_out = mem_read_in;
                  read_state = STATE_2;
            end else if(read_state == STATE_2)begin    
                  counter_s1 = counter_s1-6'd1;
                  read_state = STATE_RESET;
            end else begin
                if(counter_s1!=6'd0)begin
                 read_state = STATE_INITIAL;
                 words_done = 1'd0;
                end else begin
                    words_done = 1'd1;
                end
                RorW = 2'd0;
                SE = 1'd0;
            end
        end else if(Read_or_Write == SENSOR2_READ)begin     
            if(read_state == STATE_INITIAL)begin
                if(next_word)begin
                  mem_sel = 3'd4;
                  RorW = 2'b01;
                  mem_address = counter_s2-6'd1;
                  PC_B = 1'd0;
                  read_state = STATE_1;
                end
            end else if(read_state == STATE_1)begin
                  PC_B = 1'd1;
                  SE = 1'd1;
                  tx_out = mem_read_in;
                  read_state = STATE_2;
            end else if(read_state == STATE_2)begin
                  counter_s2 = counter_s2-6'd1;                 
                  read_state = STATE_RESET;
            end else begin
                if(counter_s2!=0)begin
                 read_state = STATE_INITIAL;
                 words_done = 1'd0;
                end else begin
                    words_done = 1'd1;
                end
                RorW = 2'd0;
                SE = 1'd0;
            end
        end
    end
end//always
endmodule
