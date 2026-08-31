`timescale 1ns/1ps

// Drives usb_rx with bit-banged UART frames and checks both layers:
// raw byte reconstruction (uart_rx0.valid/data, via the byte_count/last_byte
// monitor below) and the command-frame parser's accept/reject behavior
// (cmd_ping_seen). Malformed-frame tests run before the one valid PING so
// a false-positive ping_seen from any of them is unambiguous.
module tb_uart_rx;

localparam BIT_NS=8680; // 1/115200 baud @ 50 MHz sys_clk, matches tb_uart.v

// mirrors the RESP_* values in rtl/top.v -- kept local rather than reaching
// into the DUT's localparams so this file still reads standalone
localparam TB_RESP_ACK               = 8'h00;
localparam TB_RESP_NACK_BAD_CHECKSUM = 8'h01;
localparam TB_RESP_NACK_BAD_VERSION  = 8'h02;
localparam TB_RESP_NACK_UNKNOWN_CMD  = 8'h03;

reg clk=0;
reg usb_rx=1;
wire [7:0] led;
wire usb_tx;

always #5 clk=~clk;

top dut (
    .clk(clk),
    .led(led),
    .usb_tx(usb_tx),
    .usb_rx(usb_rx)
);

integer byte_count=0;
reg [7:0] last_byte=0;

always @(posedge dut.rx_valid) begin
    byte_count=byte_count+1;
    last_byte=dut.rx_data;
end

// independent count of how many times each command pulse actually fired,
// so "executes once per valid frame" and "malformed frames are inert" can
// be checked as exact totals rather than just spot-checked after each test
integer reset_pulse_count=0;
integer snapshot_pulse_count=0;

always @(posedge dut.cmd_reset_counters)
    reset_pulse_count=reset_pulse_count+1;

always @(posedge dut.cmd_capture_snapshot)
    snapshot_pulse_count=snapshot_pulse_count+1;

// TX-side packet monitor: decodes whatever comes out of usb_tx -- telemetry
// (magic 0xA5 0x5A) or response (0xA5 0x5B) -- independently of what the
// main test thread is doing on usb_rx. Runs for the whole simulation so
// "does a command sent mid-telemetry get lost or interleaved" is observed
// on the wire, not just inferred from internal state. Telemetry has no
// checksum of its own (pre-existing protocol, unchanged here); response
// checksum is verified against the same XOR rule the RTL uses.
integer telemetry_count=0;
integer response_count=0;
integer tx_error_count=0;

reg [7:0] last_telemetry_flags=0;

reg [7:0] last_resp_cmd_echo=0;
reg [7:0] last_resp_status=0;
reg [31:0] last_resp_data=0;
reg last_resp_checksum_ok=0;

event response_received;
event telemetry_received;

task tx_recv_byte(output [7:0] b);
    integer i;
    begin
        @(negedge usb_tx);
        #(BIT_NS + BIT_NS/2);
        for(i=0;i<8;i=i+1) begin
            b[i]=usb_tx;
            #(BIT_NS);
        end
        #(BIT_NS/2); // clear of the next start bit before looping back
    end
endtask

initial begin : tx_monitor
    reg [7:0] b0, b1, ver;
    reg [7:0] flags, cmd_echo, status, chksum, calc_chksum;
    reg [7:0] d0,d1,d2,d3;
    reg [7:0] junk;
    integer i;

    forever begin
        tx_recv_byte(b0);

        if(b0!==8'hA5) begin
            tx_error_count=tx_error_count+1;
            $display("  [tx_monitor] t=%0t bad first byte 0x%02h", $time, b0);
        end
        else begin
            tx_recv_byte(b1);

            if(b1===8'h5A) begin
                tx_recv_byte(ver);
                tx_recv_byte(flags);

                for(i=0;i<20;i=i+1) // CYCLE(8)+INSTR(4)+MEM(4)+MMIO(4), don't need the values here
                    tx_recv_byte(junk);

                if(ver!==8'h01) begin
                    tx_error_count=tx_error_count+1;
                    $display("  [tx_monitor] t=%0t telemetry bad version 0x%02h", $time, ver);
                end

                last_telemetry_flags=flags;
                telemetry_count=telemetry_count+1;
                ->telemetry_received;
            end
            else if(b1===8'h5B) begin
                tx_recv_byte(ver);
                tx_recv_byte(cmd_echo);
                tx_recv_byte(status);
                tx_recv_byte(d0);
                tx_recv_byte(d1);
                tx_recv_byte(d2);
                tx_recv_byte(d3);
                tx_recv_byte(chksum);

                calc_chksum=ver^cmd_echo^status^d0^d1^d2^d3;

                if(ver!==8'h01) begin
                    tx_error_count=tx_error_count+1;
                    $display("  [tx_monitor] t=%0t response bad version 0x%02h", $time, ver);
                end

                last_resp_cmd_echo=cmd_echo;
                last_resp_status=status;
                last_resp_data={d3,d2,d1,d0};
                last_resp_checksum_ok=(chksum===calc_chksum);

                response_count=response_count+1;
                ->response_received;
            end
            else begin
                tx_error_count=tx_error_count+1;
                $display("  [tx_monitor] t=%0t bad second byte 0x%02h (after valid 0xA5)", $time, b1);
            end
        end
    end
end

task send_byte(input [7:0] b);
    integer i;
    begin
        usb_rx=0; #(BIT_NS);
        for(i=0;i<8;i=i+1) begin
            usb_rx=b[i];
            #(BIT_NS);
        end
        usb_rx=1; #(BIT_NS);
    end
endtask

task send_frame(input [7:0] version, input [7:0] cmd, input [31:0] arg, input [7:0] checksum);
    begin
        send_byte(8'hA5);
        send_byte(8'h5A);
        send_byte(version);
        send_byte(cmd);
        send_byte(arg[7:0]);
        send_byte(arg[15:8]);
        send_byte(arg[23:16]);
        send_byte(arg[31:24]);
        send_byte(checksum);
    end
endtask

// Waits for exactly one new response and checks its fields. 1.5ms is
// generous when nothing else is queued ahead of it (worst case with a
// telemetry packet in the way is closer to 3ms, handled separately by the
// telemetry-coexistence test below with its own wait).
task expect_response(
    input [7:0] want_cmd_echo,
    input [7:0] want_status,
    input check_data,
    input [31:0] want_data
);
    integer resp_before;
    begin
        resp_before=response_count;
        #(1_500_000);

        if(response_count!==resp_before+1) begin
            $display("  FAIL: expected exactly 1 new response, response_count %0d->%0d", resp_before, response_count);
            errors=errors+1;
        end
        else begin
            $display("  observed: cmd_echo=0x%02h status=0x%02h data=%0d checksum_ok=%0d",
                last_resp_cmd_echo, last_resp_status, last_resp_data, last_resp_checksum_ok);

            if(last_resp_cmd_echo!==want_cmd_echo) begin
                $display("  FAIL: cmd_echo mismatch, expected 0x%02h", want_cmd_echo);
                errors=errors+1;
            end

            if(last_resp_status!==want_status) begin
                $display("  FAIL: status mismatch, expected 0x%02h", want_status);
                errors=errors+1;
            end

            if(!last_resp_checksum_ok) begin
                $display("  FAIL: response checksum invalid");
                errors=errors+1;
            end

            if(check_data && last_resp_data!==want_data) begin
                $display("  FAIL: data mismatch, expected %0d", want_data);
                errors=errors+1;
            end
        end
    end
endtask

integer errors;

initial begin
    $dumpfile("sim/wave_uart_rx.vcd");
    $dumpvars(0, tb_uart_rx);

    errors=0;

    #3000; // reset_counter releases resetn ~2560ns in

    $display("test: garbage bytes before any real frame");
    send_byte(8'h00);
    send_byte(8'hFF);
    send_byte(8'h37);
    #(2*BIT_NS);

    if(dut.cmd_ping_seen!==1'b0) begin
        $display("  FAIL: cmd_ping_seen set by garbage bytes");
        errors=errors+1;
    end else
        $display("  OK: cmd_ping_seen still 0, byte_count=%0d", byte_count);

    $display("test: bad magic (second byte wrong)");
    send_byte(8'hA5);
    send_byte(8'h00); // should have been 0x5A
    send_byte(8'h01);
    #(2*BIT_NS);

    if(dut.cmd_ping_seen!==1'b0) begin
        $display("  FAIL: cmd_ping_seen set after bad magic");
        errors=errors+1;
    end else
        $display("  OK: cmd_ping_seen still 0");

    $display("test: bad version");
    send_byte(8'hA5);
    send_byte(8'h5A);
    send_byte(8'h02); // unsupported version
    send_byte(8'h01);
    send_byte(8'h00);
    #(2*BIT_NS);

    if(dut.cmd_ping_seen!==1'b0) begin
        $display("  FAIL: cmd_ping_seen set after bad version");
        errors=errors+1;
    end else
        $display("  OK: cmd_ping_seen still 0");

    $display("test: bad checksum, otherwise well-formed PING");
    send_frame(8'h01, 8'h01, 32'h00000000, 8'hFF); // correct checksum is 0x00
    #(2*BIT_NS);

    if(dut.cmd_ping_seen!==1'b0) begin
        $display("  FAIL: cmd_ping_seen set despite bad checksum");
        errors=errors+1;
    end else
        $display("  OK: cmd_ping_seen still 0");

    $display("test: incomplete packet, then silence past the RX timeout");
    send_byte(8'hA5);
    send_byte(8'h5A);
    send_byte(8'h01); // version only, then nothing else arrives

    #450000; // > CMD_TIMEOUT_CYCLES (20000 cycles = 400us @ 50MHz)

    if(dut.cmd_state!==0) begin
        $display("  FAIL: parser stuck mid-frame after timeout (state=%0d)", dut.cmd_state);
        errors=errors+1;
    end else
        $display("  OK: parser back at RX_IDLE after timeout");

    $display("test: valid PING after all of the above");
    send_frame(8'h01, 8'h01, 32'h00000000, 8'h00);
    #(2*BIT_NS);

    if(dut.cmd_ping_seen===1'b1)
        $display("  OK: cmd_ping_seen set");
    else begin
        $display("  FAIL: cmd_ping_seen not set after valid PING");
        errors=errors+1;
    end

    $display("test: RESET_COUNTERS drops instr_count and it resumes counting");
    begin : reset_counters_test
        reg [31:0] pre_reset_instr;
        reg [31:0] post_reset_instr;
        reg [31:0] resumed_instr;

        #(20*BIT_NS); // let the CPU run a while so instr_count is well above 0
        pre_reset_instr=dut.instr_count;

        if(pre_reset_instr==0) begin
            $display("  FAIL: instr_count was 0 before reset, test is meaningless");
            errors=errors+1;
        end

        send_frame(8'h01, 8'h02, 32'h00000000, 8'h03); // RESET_COUNTERS, checksum=VER^CMD
        post_reset_instr=dut.instr_count;

        // the CPU keeps fetching during the ~86us frame transmission, so
        // post_reset_instr won't be exactly 0 -- it only has to be far
        // below where it would have landed without the reset
        if(post_reset_instr>=pre_reset_instr) begin
            $display("  FAIL: instr_count did not drop (pre=%0d post=%0d)", pre_reset_instr, post_reset_instr);
            errors=errors+1;
        end else
            $display("  OK: instr_count dropped (pre=%0d post=%0d)", pre_reset_instr, post_reset_instr);

        #(20*BIT_NS);
        resumed_instr=dut.instr_count;

        if(resumed_instr<=post_reset_instr) begin
            $display("  FAIL: instr_count did not resume counting (post=%0d resumed=%0d)", post_reset_instr, resumed_instr);
            errors=errors+1;
        end else
            $display("  OK: instr_count resumed counting (post=%0d resumed=%0d)", post_reset_instr, resumed_instr);
    end

    $display("test: CAPTURE_SNAPSHOT freezes coherent snapshot registers");
    begin : snapshot_test
        reg [31:0] snap_instr_at_capture;

        send_frame(8'h01, 8'h03, 32'h00000000, 8'h02); // CAPTURE_SNAPSHOT, checksum=VER^CMD
        #(5*BIT_NS);

        snap_instr_at_capture=dut.snap_instr_count;

        if(snap_instr_at_capture==0) begin
            $display("  FAIL: snap_instr_count still 0 after CAPTURE_SNAPSHOT");
            errors=errors+1;
        end else
            $display("  OK: snap_instr_count captured (%0d)", snap_instr_at_capture);

        #(30*BIT_NS); // let the live counter run well past the frozen value

        if(dut.instr_count<=snap_instr_at_capture) begin
            $display("  FAIL: live instr_count did not advance past the snapshot");
            errors=errors+1;
        end

        if(dut.snap_instr_count!==snap_instr_at_capture) begin
            $display("  FAIL: snap_instr_count changed after capture (was %0d, now %0d)", snap_instr_at_capture, dut.snap_instr_count);
            errors=errors+1;
        end else
            $display("  OK: snap_instr_count stayed frozen while live count kept advancing");
    end

    $display("test: bad checksum on RESET_COUNTERS causes no reset");
    begin : bad_checksum_reset_test
        reg [31:0] before_instr;
        before_instr=dut.instr_count;

        send_frame(8'h01, 8'h02, 32'h00000000, 8'hFF); // wrong checksum
        #(2*BIT_NS);

        if(dut.instr_count<before_instr) begin
            $display("  FAIL: instr_count dropped despite bad checksum");
            errors=errors+1;
        end else
            $display("  OK: instr_count unaffected");
    end

    $display("test: bad checksum on CAPTURE_SNAPSHOT causes no capture");
    begin : bad_checksum_snapshot_test
        reg [31:0] before_snap;
        before_snap=dut.snap_instr_count;

        send_frame(8'h01, 8'h03, 32'h00000000, 8'hFF); // wrong checksum
        #(2*BIT_NS);

        if(dut.snap_instr_count!==before_snap) begin
            $display("  FAIL: snap_instr_count changed despite bad checksum");
            errors=errors+1;
        end else
            $display("  OK: snap_instr_count unaffected");
    end

    $display("test: unsupported command ID with a valid checksum is a no-op");
    begin : unsupported_cmd_test
        reg [31:0] before_instr;
        reg [31:0] before_snap;
        before_instr=dut.instr_count;
        before_snap=dut.snap_instr_count;

        send_frame(8'h01, 8'hFF, 32'h00000000, 8'hFE); // cmd=0xFF, checksum correct for this frame
        #(2*BIT_NS);

        if(dut.instr_count<before_instr) begin
            $display("  FAIL: instr_count dropped on unsupported command");
            errors=errors+1;
        end

        if(dut.snap_instr_count!==before_snap) begin
            $display("  FAIL: snapshot changed on unsupported command");
            errors=errors+1;
        end else
            $display("  OK: unsupported command produced no side effect");
    end

    $display("test: partial RESET_COUNTERS frame + timeout causes no reset");
    begin : partial_reset_test
        reg [31:0] before_instr;
        before_instr=dut.instr_count;

        send_byte(8'hA5);
        send_byte(8'h5A);
        send_byte(8'h01);
        send_byte(8'h02); // CMD byte only, frame abandoned here
        #450000;

        if(dut.instr_count<before_instr) begin
            $display("  FAIL: instr_count dropped after an abandoned frame");
            errors=errors+1;
        end else
            $display("  OK: instr_count unaffected by the abandoned frame");
    end

    $display("test: a second valid RESET_COUNTERS after all the malformed traffic above");
    begin : second_reset_test
        reg [31:0] pre2, post2;
        #(20*BIT_NS);
        pre2=dut.instr_count;

        send_frame(8'h01, 8'h02, 32'h00000000, 8'h03);
        post2=dut.instr_count;

        if(post2>=pre2) begin
            $display("  FAIL: second RESET_COUNTERS did not drop instr_count (pre=%0d post=%0d)", pre2, post2);
            errors=errors+1;
        end else
            $display("  OK: second RESET_COUNTERS also dropped instr_count (pre=%0d post=%0d)", pre2, post2);
    end

    // the M1/M2-era tests above sent several frames without waiting for
    // (or knowing about) the responses they now also trigger -- drain
    // whatever's left pending before the M3 tests start counting exactly
    // one response per command
    #(3_500_000);

    $display("test: valid PING produces an explicit ACK response");
    send_frame(8'h01, 8'h01, 32'h00000000, 8'h00);
    expect_response(8'h01, TB_RESP_ACK, 0, 32'b0);

    $display("test: valid RESET_COUNTERS produces an explicit ACK response");
    send_frame(8'h01, 8'h02, 32'h00000000, 8'h03);
    expect_response(8'h02, TB_RESP_ACK, 0, 32'b0);

    $display("test: valid CAPTURE_SNAPSHOT produces an ACK carrying snap_instr_count");
    send_frame(8'h01, 8'h03, 32'h00000000, 8'h02);
    expect_response(8'h03, TB_RESP_ACK, 0, 32'b0); // data checked against the RTL register below

    if(last_resp_data!==dut.snap_instr_count) begin
        $display("  FAIL: response DATA (%0d) doesn't match snap_instr_count (%0d)", last_resp_data, dut.snap_instr_count);
        errors=errors+1;
    end else
        $display("  OK: response DATA matches the frozen snap_instr_count (%0d)", dut.snap_instr_count);

    $display("test: bad checksum produces NACK_BAD_CHECKSUM, no side effect");
    begin : m3_bad_checksum
        reg [31:0] before_instr;
        before_instr=dut.instr_count;

        send_frame(8'h01, 8'h01, 32'h00000000, 8'hFF); // PING, wrong checksum
        expect_response(8'h01, TB_RESP_NACK_BAD_CHECKSUM, 0, 32'b0);

        if(dut.instr_count<before_instr || dut.cmd_ping_seen!==1'b1) begin
            // cmd_ping_seen is sticky from the earlier valid PING, so this
            // just confirms nothing regressed -- the real proof is instr_count
        end
    end

    $display("test: unsupported command (valid checksum) produces NACK_UNKNOWN_CMD");
    send_frame(8'h01, 8'hFF, 32'h00000000, 8'hFE);
    expect_response(8'hFF, TB_RESP_NACK_UNKNOWN_CMD, 0, 32'b0);

    $display("test: bad version produces NACK_BAD_VERSION with cmd_echo=0x00");
    send_byte(8'hA5);
    send_byte(8'h5A);
    send_byte(8'h02); // unsupported version, frame abandoned here
    expect_response(8'h00, TB_RESP_NACK_BAD_VERSION, 0, 32'b0);

    $display("test: garbage and a timed-out partial frame produce no response");
    begin : m3_no_response_test
        integer resp_before;
        resp_before=response_count;

        send_byte(8'h11);
        send_byte(8'h22);
        send_byte(8'h33);

        send_byte(8'hA5);
        send_byte(8'h5A);
        send_byte(8'h01);
        send_byte(8'h02); // CMD byte only, then silence
        #450000;

        if(response_count!==resp_before) begin
            $display("  FAIL: a response was sent for noise/timeout (count %0d->%0d)", resp_before, response_count);
            errors=errors+1;
        end else
            $display("  OK: no response sent for noise or an abandoned frame");
    end

    $display("test: a valid command still works right after that noise");
    send_frame(8'h01, 8'h01, 32'h00000000, 8'h00);
    expect_response(8'h01, TB_RESP_ACK, 0, 32'b0);

    $display("test: command arriving mid-telemetry is queued, not lost or interleaved");
    begin : m3_telemetry_coexist
        integer telemetry_before, response_before, errors_before;

        telemetry_before=telemetry_count;
        response_before=response_count;
        errors_before=tx_error_count;

        // fast-forward the free-running 1s telemetry timer instead of
        // waiting for it in real sim time -- this only nudges the counter,
        // the RTL's own increment/compare logic is what actually fires it
        force dut.uart_interval=49999990;
        #100;
        release dut.uart_interval;
        #500; // let it tick the last few cycles and enter uart_state==1

        if(dut.uart_state==0) begin
            $display("  FAIL: telemetry didn't start after forcing uart_interval");
            errors=errors+1;
        end

        send_frame(8'h01, 8'h01, 32'h00000000, 8'h00); // PING, arrives while telemetry is still sending

        @(telemetry_received);
        if(telemetry_count!==telemetry_before+1) begin
            $display("  FAIL: telemetry_received fired but count is wrong");
            errors=errors+1;
        end

        if(response_count!==response_before) begin
            $display("  FAIL: response bytes appeared before telemetry finished (interleaved)");
            errors=errors+1;
        end else
            $display("  OK: telemetry packet completed with no response bytes mixed in");

        @(response_received);
        if(response_count!==response_before+1) begin
            $display("  FAIL: response_received fired but count is wrong");
            errors=errors+1;
        end

        if(last_resp_cmd_echo!==8'h01 || last_resp_status!==TB_RESP_ACK || !last_resp_checksum_ok) begin
            $display("  FAIL: post-telemetry response malformed (cmd_echo=0x%02h status=0x%02h ok=%0d)",
                last_resp_cmd_echo, last_resp_status, last_resp_checksum_ok);
            errors=errors+1;
        end else
            $display("  OK: response sent right after telemetry, correctly formed");

        if(tx_error_count!==errors_before) begin
            $display("  FAIL: tx_error_count increased (%0d->%0d) -- framing broke somewhere", errors_before, tx_error_count);
            errors=errors+1;
        end
    end

    $display("test: two valid commands sent back to back");
    begin : m3_back_to_back
        integer response_before;
        response_before=response_count;

        send_frame(8'h01, 8'h01, 32'h00000000, 8'h00); // PING
        send_frame(8'h01, 8'h02, 32'h00000000, 8'h03); // RESET_COUNTERS, no gap

        #(3_000_000);

        // documented one-outstanding-response rule: if the first response
        // hadn't started transmitting before the second command classified,
        // it gets overwritten and only the second is ever sent. Either
        // outcome is correct -- report which one actually happened.
        if(response_count==response_before+2) begin
            $display("  OK: both commands got independent responses (%0d total)", response_count-response_before);
        end
        else if(response_count==response_before+1) begin
            $display("  OK: second command's response overwrote the first's (documented one-outstanding rule), last cmd_echo=0x%02h",
                last_resp_cmd_echo);
        end
        else begin
            $display("  FAIL: unexpected response_count delta (%0d)", response_count-response_before);
            errors=errors+1;
        end
    end

    if(dut.trap!==1'b0) begin
        $display("  FAIL: trap asserted during M3 testing");
        errors=errors+1;
    end else
        $display("  OK: trap stayed 0 throughout");

    if(reset_pulse_count!==4) begin
        $display("  FAIL: expected exactly 4 cmd_reset_counters pulses (2 M2 + 2 M3 valid frames), saw %0d", reset_pulse_count);
        errors=errors+1;
    end else
        $display("  OK: cmd_reset_counters pulsed exactly 4 times, matching the 4 valid RESET_COUNTERS frames sent");

    if(snapshot_pulse_count!==2) begin
        $display("  FAIL: expected exactly 2 cmd_capture_snapshot pulses (1 M2 + 1 M3 valid frame), saw %0d", snapshot_pulse_count);
        errors=errors+1;
    end else
        $display("  OK: cmd_capture_snapshot pulsed exactly twice, matching the 2 valid CAPTURE_SNAPSHOT frames sent");

    if(byte_count!==181) begin
        $display("  FAIL: byte-level receiver saw %0d bytes, expected 181", byte_count);
        errors=errors+1;
    end else
        $display("  OK: byte-level receiver reconstructed all 181 transmitted bytes");

    if(errors==0)
        $display(">>> UART RX PASS");
    else
        $display(">>> UART RX FAIL: %0d error(s)", errors);

    $finish;
end

endmodule
