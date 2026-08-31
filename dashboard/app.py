from flask import Flask,jsonify,render_template

from serial_reader import TelemetryReader


PORT="/dev/cu.usbserial-FT4MG9OV1"

app=Flask(__name__)

telemetry=TelemetryReader(PORT)
telemetry.start()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/telemetry")
def telemetry_api():
    return jsonify(telemetry.snapshot())


if __name__=="__main__":
    app.run(
        host="127.0.0.1",
        port=5000,
        debug=False
    )