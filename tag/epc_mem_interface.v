`timescale 1ns / 1ps

module epc_memorymodule
#(
    parameter data_width = 16
   // parameter SEL_SIZE = 61
)
(
    input wire clk,reset,curr_sl_flag,curr_inven_flag,
     
    input wire [12:0] rx_cmd,
    //input wire [65:0]we,
    input wire [1:0] readwritebank,
    input wire [7:0] readwriteptr, 
    input wire [7:0] readwords,
    input wire writedataout,
    
    //input wire [SEL_SIZE-1:0]select,
    input wire [2:0] sel_target,
    input wire [2:0] sel_action,
    input wire [7:0] sel_ptr,
    input wire [7:0] sel_masklen,
    input wire [15: 0] mask,
    input wire     truncate, 
    
    output reg [data_width-1:0] dout,
    output reg sl_flag,inven_flag,
    output reg [1:0]session
);

reg [data_width-1:0] EPC[0:32]; // master memory array
wire [527:0]total; // serial memory storage - 33*16 bits
reg [data_width-1:0] StoredCRC, StoredPC; // the first two 16-bit words of EPC
wire [data_width-1:0] code[0:30];//the epc codewords, so excluding crc and pc
wire CRC_clk; //clk for calculating stored crc
reg CRC_en; //enable for crc
wire [data_width-1:0] CRC,PC; 
//CRC is the o/p wire driven by crc module
//PC : the wire connected to second word of epc

//reg [7:0] length_read, pointer_write, pointer_read, length_select, pointer_select;
//length_read - 8 bit code that stores the number of words to be read
//pointer_write - for write: 8 bit ebv pointer which stores the index at which we want to write.
//                for example, if I want to write data to EPC's 3rd word. pointer_write will be
//                00000010.
//pointer_read - similar to pointer_write
//length_select - 8 bit code that stores the bit length of mask, assuming to be fixed to 16
//pointer_select - ebv pointer that provides starting bit (not word) address for the Mask 
//                 comparison.
reg [15:0] length_write;//the write data to be written
//wire [7:0] total_w,select_w;
reg [7:0] l;
reg [4:0]word_count;
reg CRC_in;

genvar p,q;
for(p=0; p<33; p=p+1)begin
    for(q=15; q>=0; q=q-1)begin
        assign total[16*(32-p) + q] = EPC[p][q];
    end
end


assign PC = EPC[1];//the wire connected to second word of epc


reg [3:0]k;
reg [3:0]j;
integer w,b,l_r,count;

always@(*)begin // asynchronous assignment to the registers storedpc and storedcrc
    StoredPC = PC;
    StoredCRC = CRC;
    EPC[0] = CRC;// we need CRC to be stored in first word
end

genvar i;
for(i=0; i<31; i=i+1) begin
    assign code[i] = EPC[i+2];
end

always@(posedge CRC_clk)begin
    if(j < (EPC[1][data_width-1:data_width-5]+5'd1))begin  
        k = k+1;
        CRC_in = EPC[j][k];
        if(k == 4'd15)begin
            j = j+1;
        end
    end
end

assign CRC_clk = (clk & CRC_en);
crc16check c(reset,CRC_clk,CRC_in,CRC);
//crc16check(reset, crcinclk, crcbitin, crc)

integer r;
integer m=0;
integer a,l_s;
integer y;
integer x;

always@(posedge clk or posedge reset)begin
    if(reset)begin
        CRC_en = 0;
        dout = 0;
        k = 0;j = 5'd1;
        CRC_in = 0;
        word_count =0;
        sl_flag = 0;
        inven_flag = 0;
        session = 0;
        l = 0;
    end else begin
        y = 31;
        
        for(a = 0;a<31;a=a+1)begin
            if(code[a] == 16'd0)begin
                y = y - 1;
            end
        end
        
        word_count = y;
        EPC[1][15:11]= y;
        EPC[1][10] = 1'd1; //fixed by tag manufacturer
        EPC[1][9:0]= 10'd0;
    
        if(EPC[1]!=0) CRC_en = 1;        
        if(j>= y+1)CRC_en = 0;
   
        if(rx_cmd[4])begin//if command is select
            if(readwritebank == 2'b01)begin // if membank is 01
                case(sel_target)
                  3'b000: session = 2'b00;
                  3'b001: session = 2'b01;
                  3'b010: session = 2'b10;
                  3'b011: session = 2'b11;
                  3'b100: sl_flag = 1'b1;
                endcase
                l_s = sel_masklen;
                l=0;
                
                while(l_s > 8'd0)begin   
                    if(total[527-sel_ptr-l]== mask[15-l])begin
                        l=l+1;
                    end
                    l_s = l_s -1;
                end
            
                if(l==sel_masklen)begin // if tag is matching
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
            end //membank
        end//select command
        l_r = 0;
        
           //read command code, still yet to modified
           if(rx_cmd[7])begin
              if(readwritebank == 2'b01)begin
                  for(w=0;w<33;w=w+1)begin
                      for(b=0;b<33;b=b+1)begin
                          if(readwriteptr == w && readwords == b)begin
                              while(l_r <= b)begin
                                  dout = EPC[w+l_r-1];
                                  l_r = l_r+1;
                              end
                          end 
                      end 
                  end 
              end 
           end
    
           //write command code, still yet to be modified 
           if(rx_cmd[8])begin
              if(readwritebank == 2'b01)begin
                  length_write = writedataout; 
                  EPC[readwriteptr] = length_write;
              end 
           end
   
    end//reset
end//always



endmodule

