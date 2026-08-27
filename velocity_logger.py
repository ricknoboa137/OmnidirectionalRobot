"""Log the wheel velocities streamed by ESP32_MotorDriver and plot the controller response.

The sketch publishes CSV rows on the topic "motor_telemetry":

    seq,t_ms,setA,velA,pwmA,setB,velB,pwmB,setC,velC,pwmC

    seq   row counter produced by the ESP32 (used here to detect lost packets)
    t_ms  millis() on the ESP32 when the sample was taken
    setX  commanded wheel linear velocity  [cm/s]
    velX  measured wheel linear velocity   [cm/s], filtered, signed
    pwmX  PID output, signed by the commanded direction  [0-255]

Usage
    python velocity_logger.py                     # log to logs/velocities_<timestamp>.csv
    python velocity_logger.py --plot              # plot the log when you stop it with Ctrl+C
    python velocity_logger.py --broker 192.168.0.110 --duration 20 --plot
    python velocity_logger.py --replay logs/velocities_2026....csv   # just plot an old log

Requires paho-mqtt ( pip install paho-mqtt ); plotting also needs matplotlib.
"""

import argparse
import csv
import os
import random
import sys
import time

TELEMETRY_TOPIC = "motor_telemetry"
CONTROL_TOPIC = "motor_telemetry_ctrl"
COLUMNS = ["seq", "t_ms",
           "setA", "velA", "pwmA",
           "setB", "velB", "pwmB",
           "setC", "velC", "pwmC"]

###############################################################################


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--broker", default="192.168.0.153", help="MQTT broker IP")
    p.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    p.add_argument("--out", default=None, help="output CSV file (default: logs/velocities_<timestamp>.csv)")
    p.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = until Ctrl+C)")
    p.add_argument("--plot", action="store_true", help="plot the log once the recording stops")
    p.add_argument("--replay", default=None, help="skip recording, just plot this CSV file")
    return p.parse_args()

###############################################################################


class Recorder:
    """Collects the CSV rows published by the ESP32 and writes them to a file."""

    def __init__(self, path):
        self.path = path
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["host_time"] + COLUMNS)
        self.rows = 0
        self.dropped = 0
        self.next_seq = None
        self.t0 = time.time()

    def feed(self, payload):
        # one MQTT message carries a batch of rows sampled by the control task
        for line in payload.decode("utf-8", "replace").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):   # header line republished on reconnect
                continue
            fields = line.split(",")
            if len(fields) != len(COLUMNS):
                continue
            try:
                seq = int(fields[0])
            except ValueError:
                continue
            if self.next_seq is not None and seq > self.next_seq:
                self.dropped += seq - self.next_seq   # gap in the sequence = lost packet
            self.next_seq = seq + 1
            self.writer.writerow([f"{time.time() - self.t0:.4f}"] + fields)
            self.rows += 1

    def close(self):
        self.file.close()

###############################################################################


def record(args):
    from paho.mqtt import client as mqtt_client

    out = args.out
    if out is None:
        os.makedirs("logs", exist_ok=True)
        out = os.path.join("logs", time.strftime("velocities_%Y%m%d_%H%M%S.csv"))

    rec = Recorder(out)

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to MQTT broker {args.broker}:{args.port}")
            client.subscribe(TELEMETRY_TOPIC)
            client.publish(CONTROL_TOPIC, "1")     # make sure the ESP32 is streaming
        else:
            print(f"Failed to connect, return code {rc}")

    def on_message(client, userdata, msg):
        rec.feed(msg.payload)

    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1,
                                f"VelocityLogger-{random.randint(0, 1000)}")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.broker, args.port)
    client.loop_start()

    print(f"Logging to {out} - press Ctrl+C to stop")
    start = time.time()
    last_report, last_rows = start, 0
    try:
        while True:
            time.sleep(0.5)
            now = time.time()
            if now - last_report >= 2.0:
                rate = (rec.rows - last_rows) / (now - last_report)
                print(f"  {rec.rows} samples  ({rate:.0f} Hz)  lost: {rec.dropped}")
                last_report, last_rows = now, rec.rows
            if args.duration and (now - start) >= args.duration:
                break
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        client.loop_stop()
        client.disconnect()
        rec.close()

    print(f"Wrote {rec.rows} samples to {out} ({rec.dropped} lost in transit)")
    return out

###############################################################################


def plot(path):
    import matplotlib.pyplot as plt

    data = {c: [] for c in COLUMNS}
    host_time = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            host_time.append(float(row["host_time"]))
            for c in COLUMNS:
                data[c].append(float(row[c]))

    if not host_time:
        print(f"{path} contains no samples")
        return

    # ESP32 timebase, zeroed on the first sample, is the cleanest x axis
    t0 = data["t_ms"][0]
    t = [(v - t0) / 1000.0 for v in data["t_ms"]]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    for ax, wheel in zip(axes, ("A", "B", "C")):
        ax.plot(t, data["set" + wheel], label="setpoint [cm/s]", linestyle="--")
        ax.plot(t, data["vel" + wheel], label="measured [cm/s]")
        ax.set_ylabel(f"wheel {wheel}")
        ax.grid(True, alpha=0.3)
        pwm = ax.twinx()                       # PID output on its own axis
        pwm.plot(t, [v / 255.0 for v in data["pwm" + wheel]], color="grey", alpha=0.5, label="PID output (norm.)")
        pwm.set_ylabel("PWM (norm.)", color="grey", fontsize=8)
        pwm.set_ylim(-1.05, 1.05)
        lines = ax.get_lines() + pwm.get_lines()
        ax.legend(lines, [l.get_label() for l in lines], loc="upper right", fontsize=8)
    axes[-1].set_xlabel("time [s]")
    axes[0].set_title(os.path.basename(path))
    fig.tight_layout()
    plt.show()

###############################################################################

if __name__ == "__main__":
    args = parse_args()
    if args.replay:
        plot(args.replay)
    else:
        path = record(args)
        if args.plot:
            plot(path)
