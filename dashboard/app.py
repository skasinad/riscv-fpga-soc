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

#transport failures (send_command's "error" values) vs fpga nacks are two
#different things - a nack means the fpga actually answered, so that's a
#200 with ok:false. a transport failure means there's no real answer
_ERROR_STATUS_CODES={"busy":429}


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/telemetry")
def telemetry_api():
    return jsonify(telemetry.snapshot())


@app.route("/api/events")
def events_api():
    return jsonify({"events":telemetry.get_events()})


@app.route("/api/events/clear",methods=["POST"])
def clear_events_api():
    #host-side display history only, never touches the fpga, not to be
    #confused with RESET_COUNTERS or SYSTEM_RESET
    telemetry.clear_events()
    return jsonify({"ok":True})


#one place shapes both the http response and the logged event so the
#wording can't drift between the two. log_label lets a caller log
#something more specific than command_name, like "SET_WORKLOAD MEMORY"
#instead of just "SET_WORKLOAD", without touching the json "command" field
def _command_response(command_name,result,extra=None,log_label=None):
    label=log_label if log_label is not None else command_name

    if not result["ok"]:
        code=_ERROR_STATUS_CODES.get(result["error"],503)

        if result["error"]=="busy":
            telemetry.add_event("command",f"{label} BUSY")
        elif result["error"]=="timeout":
            telemetry.add_event("command",f"{label} TIMEOUT")
        else:
            telemetry.add_event("command",f"COMMAND ERROR {result['error']}")

        return jsonify({
            "ok":False,
            "command":command_name,
            "error":result["error"]
        }),code

    status_name=RESP_STATUS_NAMES.get(result["status"],"UNKNOWN")

    if result["status"]==RESP_ACK:
        extra_text=""
        if extra and "instruction_count" in extra:
            extra_text=f"  instr={extra['instruction_count']}"

        telemetry.add_event("command",f"{label} ACK{extra_text}")
    else:
        telemetry.add_event("command",f"{label} {status_name}")

    body={
        "ok":result["status"]==RESP_ACK,
        "command":command_name,
        "status":status_name
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
        #fpga acked but echoed a different workload than we asked for,
        #call it failed instead of claiming a selection we can't confirm
        result={"ok":False,"error":"echo_mismatch"}

    return _command_response(
        "SET_WORKLOAD",
        result,
        extra={"workload":WORKLOAD_NAMES[workload_id]},
        log_label=f"SET_WORKLOAD {WORKLOAD_NAMES[workload_id]}"
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
    #this only means "armed", not "already trapped" - the real trap state
    #comes from telemetry (see dashboard.js), never from this response
    result=telemetry.send_command(CMD_INJECT_FAULT)
    return _command_response("INJECT_FAULT",result)


@app.route("/api/system-reset",methods=["POST"])
def system_reset_api():
    #send_command returns once the ack is decoded, and the rtl guarantees
    #that's already fully sent before the fpga resets (see
    #awaiting_reset_flush/reset_pulse_trigger in top.v). the reset pulse
    #never touches the ftdi/usb connection so this says nothing about
    #whether post has rerun yet, frontend polls telemetry for that
    result=telemetry.send_command(CMD_SYSTEM_RESET)
    return _command_response("SYSTEM_RESET",result)


if __name__=="__main__":
    app.run(
        host="127.0.0.1",
        port=5000,
        debug=False,
        threaded=True
    )
