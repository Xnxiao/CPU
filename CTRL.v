`include "lib/defines.vh"
module CTRL(
    input wire rst,
    input wire stall_id,
    input wire stallreq_for_ex,
    
    // output reg flush,
    // output reg [31:0] new_pc,
    output reg [`StallBus-1:0] stall
);  
    always @ (*) begin
        if (rst) begin
            stall = `StallBus'b0;
        end
        else begin
            stall = (stall_id==`Stop) ? 6'b000111:
                    (stallreq_for_ex==`Stop) ? 6'b001111:6'b000000;
        end
    end

endmodule