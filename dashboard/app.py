from flask import Flask,jsonify,render_template,request

from serial_reader import (
    TelemetryReader,
    CMD_RESET_COUNTERS,
    CMD_CAPTURE_SNAPSHOT,
    CMD_SET_WORKLOAD,
    CMD_INJECT_FAULT,
    CMD_SYSTEM_RESET,
    RESP_ACK,
    RESP_STATUS_NAMES,
    WORKLOAD_IDS,
    WORKLOAD_NAMES
)


PORT="/dev/cu.usbserial-FT4MG9OV1"

app=Flask(__name__)

telemetry=TelemetryReader(PORT)
telemetry.start()

# Transport-level failures (send_command()'s "error" values) vs FPGA-level
# NACKs are two different failure classes -- a NACK means the request made
# it to the FPGA and got a real answer, so it's a 200 with ok:false. A
# transport failure means there's no real answer to report.
_ERROR_STATUS_CODES={"busy":429}


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/telemetry")
def telemetry_api():
    return jsonify(telemetry.snapshot())


def _command_response(command_name,result,extra=None):
    if not result["ok"]:
        code=_ERROR_STATUS_CODES.get(result["error"],503)

        return jsonify({
            "ok":False,
            "command":command_name,
            "error":result["error"]
        }),code

    body={
        "ok":result["status"]==RESP_ACK,
        "command":command_name,
        "status":RESP_STATUS_NAMES.get(result["status"],"UNKNOWN")
    }

    if extra:
        body.update(extra)

    return jsonify(body)


def _resolve_workload_id(requested):
    if isinstance(requested,str):
        return WORKLOAD_IDS.get(requested.strip().upper())

    if isinstance(requested,int) and not isinstance(requested,bool):
        if requested in WORKLOAD_NAMES:
            return requested

    return None


@app.route("/api/workload",methods=["POST"])
def workload_api():
    payload=request.get_json(silent=True) or {}
    workload_id=_resolve_workload_id(payload.get("workload"))

    if workload_id is None:
        return jsonify({
            "ok":False,
            "command":"SET_WORKLOAD",
            "error":"invalid_workload"
        }),400

    result=telemetry.send_command(CMD_SET_WORKLOAD,workload_id)

    if result["ok"] and result["status"]==RESP_ACK and result["data"]!=workload_id:
        # FPGA ACKed but echoed a different workload than requested --
        # report it as failed rather than claiming a selection we can't
        # actually confirm happened
        result={"ok":False,"error":"echo_mismatch"}

    return _command_response(
        "SET_WORKLOAD",
        result,
        extra={"workload":WORKLOAD_NAMES[workload_id]}
    )


@app.route("/api/reset-counters",methods=["POST"])
def reset_counters_api():
    result=telemetry.send_command(CMD_RESET_COUNTERS)
    return _command_response("RESET_COUNTERS",result)


@app.route("/api/snapshot",methods=["POST"])
def snapshot_api():
    result=telemetry.send_command(CMD_CAPTURE_SNAPSHOT)

    extra=None
    if result["ok"] and result["status"]==RESP_ACK:
        extra={"instruction_count":result["data"]}

    return _command_response("CAPTURE_SNAPSHOT",result,extra)


@app.route("/api/inject-fault",methods=["POST"])
def inject_fault_api():
    # This ACKs "armed", not "already trapped" -- the real trap state only
    # ever comes from telemetry (see dashboard.js), never from this
    # response alone.
    result=telemetry.send_command(CMD_INJECT_FAULT)
    return _command_response("INJECT_FAULT",result)


@app.route("/api/system-reset",methods=["POST"])
def system_reset_api():
    # send_command() returns as soon as the ACK response is decoded, which
    # the RTL guarantees has already fully left the transmitter before the
    # FPGA resets (see rtl/top.v's awaiting_reset_flush/reset_pulse_trigger).
    # The reset pulse itself is purely internal to the FPGA logic -- it
    # never touches the FTDI/USB connection -- so this request completing
    # says nothing about whether POST has rerun yet; the frontend polls
    # telemetry independently for that.
    result=telemetry.send_command(CMD_SYSTEM_RESET)
    return _command_response("SYSTEM_RESET",result)


if __name__=="__main__":
    app.run(
        host="127.0.0.1",
        port=5000,
        debug=False,
        threaded=True
    )
