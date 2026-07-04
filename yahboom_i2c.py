#!/usr/bin/env python3
"""
yahboom_i2c.py - I2C sidecar for the Yahboom ROS Expansion Board (STM32 @ 0x34)

Reads battery voltage and 9-axis IMU data from the board via I2C Bus 1
and serves it as JSON on http://localhost:3001/data so that server.js can
poll it and broadcast it to the dashboard WebSocket clients.

Register map (Yahboom STM32 @ 0x34, I2C Bus 1):
  0x00 - Battery voltage   (2 bytes LE unsigned, raw/10.0 = volts)
  0x08 - Gyroscope X       (2 bytes LE int16, /3754.9 = rad/s)
  0x0A - Gyroscope Y       (2 bytes LE int16, /3754.9 = rad/s)
  0x0C - Gyroscope Z       (2 bytes LE int16, /3754.9 = rad/s)
  0x10 - Accelerometer X   (2 bytes LE int16, /1000.0 = g)
  0x12 - Accelerometer Y   (2 bytes LE int16, /1000.0 = g)
  0x14 - Accelerometer Z   (2 bytes LE int16, /1000.0 = g)
  0x18 - Magnetometer X    (2 bytes LE int16, raw LSB)
  0x1A - Magnetometer Y    (2 bytes LE int16, raw LSB)
  0x1C - Magnetometer Z    (2 bytes LE int16, raw LSB)

Usage:
  python3 yahboom_i2c.py [--bus 1] [--address 0x34] [--port 3001]
"""

import argparse
import json
import math
import os
import struct
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Lock, Thread

# Load environment variables manually from .env if present
if os.path.exists('.env'):
    with open('.env', 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#') and '=' in line:
                key, val = line.split('=', 1)
                os.environ[key.strip()] = val.strip()

# I2C Register addresses
REG_BATTERY = 0x00
REG_GYRO_X  = 0x08
REG_GYRO_Y  = 0x0A
REG_GYRO_Z  = 0x0C
REG_ACCEL_X = 0x10
REG_ACCEL_Y = 0x12
REG_ACCEL_Z = 0x14
REG_MAG_X   = 0x18
REG_MAG_Y   = 0x1A
REG_MAG_Z   = 0x1C

GYRO_RATIO  = 1.0 / 3754.9
ACCEL_RATIO = 1.0 / 1000.0

state_lock  = Lock()
latest_data = {
    "ok": False, "error": "Not yet read", "ts": 0,
    "battery": None,
    "ax": None, "ay": None, "az": None,
    "gx": None, "gy": None, "gz": None,
    "mx": None, "my": None, "mz": None,
    "roll": None, "pitch": None, "yaw": None,
}

_yaw    = 0.0
_last_ts = None


def read_int16_le(bus, addr, reg):
    raw = bus.read_i2c_block_data(addr, reg, 2)
    return struct.unpack_from('<h', bytes(raw))[0]


def poll_i2c(bus_id, board_addr, interval):
    global _yaw, _last_ts

    try:
        import smbus
    except ImportError:
        with state_lock:
            latest_data["ok"]    = False
            latest_data["error"] = "smbus not installed - run: sudo apt-get install python3-smbus"
        print("[I2C] ERROR: smbus not installed.", file=sys.stderr)
        return

    bus = None
    while True:
        try:
            if bus is None:
                bus = smbus.SMBus(bus_id)
                print(f"[I2C] Opened SMBus({bus_id}), device 0x{board_addr:02X}")

            raw_batt = bus.read_i2c_block_data(board_addr, REG_BATTERY, 2)
            batt_raw = struct.unpack_from('<H', bytes(raw_batt))[0]
            voltage  = round(batt_raw / 10.0, 1)

            gx_raw = read_int16_le(bus, board_addr, REG_GYRO_X)
            gy_raw = read_int16_le(bus, board_addr, REG_GYRO_Y)
            gz_raw = read_int16_le(bus, board_addr, REG_GYRO_Z)
            gx =  gx_raw * GYRO_RATIO
            gy = -gy_raw * GYRO_RATIO
            gz = -gz_raw * GYRO_RATIO

            ax_raw = read_int16_le(bus, board_addr, REG_ACCEL_X)
            ay_raw = read_int16_le(bus, board_addr, REG_ACCEL_Y)
            az_raw = read_int16_le(bus, board_addr, REG_ACCEL_Z)
            ax = ax_raw * ACCEL_RATIO
            ay = ay_raw * ACCEL_RATIO
            az = az_raw * ACCEL_RATIO

            mx = read_int16_le(bus, board_addr, REG_MAG_X)
            my = read_int16_le(bus, board_addr, REG_MAG_Y)
            mz = read_int16_le(bus, board_addr, REG_MAG_Z)

            pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
            roll  = math.degrees(math.atan2( ay, math.sqrt(ax*ax + az*az)))

            now = time.time()
            if _last_ts is not None:
                dt   = min(now - _last_ts, 0.1)
                _yaw += math.degrees(gz) * dt
                _yaw  = ((_yaw + 180) % 360 + 360) % 360 - 180
            _last_ts = now

            with state_lock:
                latest_data.update({
                    "ok": True, "error": None, "ts": round(now * 1000),
                    "battery": voltage,
                    "ax": round(ax, 4), "ay": round(ay, 4), "az": round(az, 4),
                    "gx": round(math.degrees(gx), 3),
                    "gy": round(math.degrees(gy), 3),
                    "gz": round(math.degrees(gz), 3),
                    "mx": mx, "my": my, "mz": mz,
                    "roll":  round(roll,  2),
                    "pitch": round(pitch, 2),
                    "yaw":   round(_yaw,  2),
                })

        except OSError as e:
            errmsg = f"I2C OSError: {e}"
            print(f"[I2C] {errmsg}", file=sys.stderr)
            with state_lock:
                latest_data["ok"]    = False
                latest_data["error"] = errmsg
            try:
                bus.close()
            except Exception:
                pass
            bus = None
            time.sleep(2.0)
            continue

        except Exception as e:
            errmsg = f"Unexpected error: {e}"
            print(f"[I2C] {errmsg}", file=sys.stderr)
            with state_lock:
                latest_data["ok"]    = False
                latest_data["error"] = errmsg

        time.sleep(interval)


class I2CHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/data", "/data/"):
            self._send(404, {"error": f"Unknown path: {self.path}"})
            return
        with state_lock:
            payload = dict(latest_data)
        self._send(200, payload)

    def _send(self, code, obj):
        body = json.dumps(obj, separators=(',', ':')).encode()
        self.send_response(code)
        self.send_header("Content-Type",                "application/json")
        self.send_header("Content-Length",              str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


def parse_args():
    p = argparse.ArgumentParser(description="Yahboom I2C sidecar")
    p.add_argument("--bus",      type=int,               default=int(os.environ.get("I2C_BUS", 1)),    help="I2C bus number")
    p.add_argument("--address",  type=lambda x: int(x,0), default=int(os.environ.get("I2C_ADDRESS", "0x34"), 0), help="I2C address")
    p.add_argument("--port",     type=int,               default=int(os.environ.get("I2C_PORT", 3001)), help="HTTP port")
    p.add_argument("--interval", type=float,             default=float(os.environ.get("I2C_INTERVAL", 0.1)),  help="Poll interval seconds")
    return p.parse_args()


def main():
    args = parse_args()
    print(f"[I2C] Yahboom I2C sidecar | Bus:{args.bus} Addr:0x{args.address:02X} Port:{args.port} Interval:{args.interval}s")
    t = Thread(target=poll_i2c, args=(args.bus, args.address, args.interval), daemon=True)
    t.start()
    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), I2CHandler)
    print(f"[I2C] Serving on http://localhost:{args.port}/data")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[I2C] Shutting down.")


if __name__ == "__main__":
    main()
