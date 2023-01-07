
// Copyright 2010 University of Washington
// License: http://creativecommons.org/licenses/by/3.0/



module counter16 (clk, reset, enable, count, overflow);

output [15:0] count;
output overflow;
input clk, reset, enable;

reg [15:0] count;

wire [15:0] next_count;
assign next_count = count + 16'd1;

wire overflow;
assign overflow = (count > 2500);

always @ (posedge clk or posedge reset) begin

  if (reset) begin
    count <= 0;
  end else if (enable & ~overflow) begin
    count <= next_count;
  end
end

endmodule

