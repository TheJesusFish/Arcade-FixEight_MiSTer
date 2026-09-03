// Persistent toggle/ack request channel. Payload stays stable until ack.
module fixeight_cdc_toggle #(
    parameter integer WIDTH = 8
) (
    input  logic             src_clk,
    input  logic             src_reset,
    input  logic             src_send,
    input  logic [WIDTH-1:0] src_payload,
    output logic             src_busy,

    input  logic             dst_clk,
    input  logic             dst_reset,
    output logic             dst_pending,
    output logic [WIDTH-1:0] dst_payload,
    input  logic             dst_ack
);

logic request_toggle;
logic acknowledge_toggle;
logic acknowledge_meta;
logic acknowledge_sync;
logic request_meta;
logic request_sync;
logic request_seen;
logic [WIDTH-1:0] payload_hold;

assign src_busy = request_toggle != acknowledge_sync;

always_ff @(posedge src_clk) begin
    if (src_reset) begin
        request_toggle <= 1'b0;
        payload_hold <= '0;
        acknowledge_meta <= 1'b0;
        acknowledge_sync <= 1'b0;
    end else begin
        acknowledge_meta <= acknowledge_toggle;
        acknowledge_sync <= acknowledge_meta;
        if (src_send && !src_busy) begin
            payload_hold <= src_payload;
            request_toggle <= ~request_toggle;
        end
    end
end

always_ff @(posedge dst_clk) begin
    if (dst_reset) begin
        request_meta <= 1'b0;
        request_sync <= 1'b0;
        request_seen <= 1'b0;
        acknowledge_toggle <= 1'b0;
        dst_pending <= 1'b0;
        dst_payload <= '0;
    end else begin
        request_meta <= request_toggle;
        request_sync <= request_meta;
        if (!dst_pending && request_sync != request_seen) begin
            dst_payload <= payload_hold;
            dst_pending <= 1'b1;
        end
        if (dst_pending && dst_ack) begin
            request_seen <= request_sync;
            acknowledge_toggle <= request_sync;
            dst_pending <= 1'b0;
        end
    end
end

endmodule
