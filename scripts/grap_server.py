#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gripper_server.py
Control Zhixing (Changingtek) industrial gripper via RS485 ModbusRTU (/dev/grap)
Author: Hency
"""

import time
from pymodbus.client.sync import ModbusSerialClient
from flask import Flask, request, jsonify

# ==============================
# Gripper Modbus Controller
# ==============================
class ZhixingGripper:
    def __init__(self, port="/dev/grap", slave_id=1, baudrate=115200):
        self.client = ModbusSerialClient(
            method="rtu",
            port=port,
            baudrate=baudrate,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=0.2
        )
        self.slave_id = slave_id
        if not self.client.connect():
            raise IOError(f"❌ Failed to connect to gripper on {port}")
        print(f"✅ Connected to gripper at {port} (ID={slave_id})")

    # --- Basic Modbus helpers ---
    def write_single(self, addr, value):
        return self.client.write_register(addr, value, slave=self.slave_id)

    def write_multiple(self, addr, values):
        return self.client.write_registers(addr, values, slave=self.slave_id)

    def read_holding(self, addr, count=1):
        return self.client.read_holding_registers(addr, count, slave=self.slave_id)

    # --- Core operations ---
    def enable(self, on=True):
        """Enable or disable the actuator"""
        self.write_single(0x0100, 1 if on else 0)
        time.sleep(0.05)

    def move_to(self, pos=500, speed=100, torque=100, accel=100):
        """Move gripper to target position (0-1000 scale typical)"""
        self.write_single(0x0104, speed)
        self.write_single(0x0105, torque)
        self.write_single(0x0106, accel)
        # Write 32-bit position (high/low)
        high = (pos >> 16) & 0xFFFF
        low = pos & 0xFFFF
        self.write_multiple(0x0102, [high, low])
        time.sleep(0.05)
        self.write_single(0x0108, 1)  # trigger
        print(f"➡️ Moving to {pos}...")

    def wait_until_done(self, timeout=5.0):
        """Wait until position reached or timeout"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            res = self.read_holding(0x0602)
            if res.isError():
                continue
            if res.registers[0] == 1:
                print("✅ Position reached")
                return True
            time.sleep(0.05)
        print("⚠️ Timeout waiting for completion")
        return False

    def open(self):
        self.move_to(0)
        self.wait_until_done()

    def close(self):
        self.move_to(8000)
        self.wait_until_done()

    def get_status(self):
        res = self.read_holding(0x0612)
        return {"alarm_code": res.registers[0] if not res.isError() else -1}

    def disconnect(self):
        self.client.close()

    def benchmark_frequency(self, samples=100):
        import time
        times = []
        for _ in range(samples):
            t0 = time.perf_counter()
            # 随便读一个寄存器，比如反馈位置 0x0609
            res = self.read_holding(0x0609, count=2)
            t1 = time.perf_counter()
            if not res.isError():
                times.append(t1 - t0)
        avg_dt = sum(times) / len(times)
        freq = 1.0 / avg_dt
        print(f"平均通信周期: {avg_dt*1000:.2f} ms  ≈  {freq:.1f} Hz")
        return freq


# ==============================
# REST API Server
# ==============================
app = Flask(__name__)
gripper = ZhixingGripper(port="/dev/grap")

@app.route("/open", methods=["POST"])
def api_open():
    gripper.enable(True)
    gripper.open()
    return jsonify({"status": "opened"})

@app.route("/close", methods=["POST"])
def api_close():
    gripper.enable(True)
    gripper.close()
    return jsonify({"status": "closed"})

@app.route("/status", methods=["GET"])
def api_status():
    return jsonify(gripper.get_status())

@app.route("/move", methods=["POST"])
def api_move():
    data = request.json or {}
    pos = int(data.get("pos", 500))
    spd = int(data.get("speed", 100))
    gripper.enable(True)
    gripper.move_to(pos, spd)
    gripper.wait_until_done()
    return jsonify({"target": pos, "status": "ok"})

if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5055)
    finally:
        gripper.disconnect()
