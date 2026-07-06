#!/usr/bin/env python3
"""HTTP :8080 + UART bridge between browser and ESP32."""

import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse

try:
    import serial
except ImportError:
    print("Install pyserial: pip3 install pyserial", file=sys.stderr)
    sys.exit(1)

ROOT = Path(__file__).resolve().parent
WEB_CONTROL = ROOT / "web_control"
WEB_MAP = ROOT / "web_map"

SERIAL_PORT = os.environ.get("ROBOT_UART", "")  # empty = auto-detect
BAUD = int(os.environ.get("ROBOT_UART_BAUD", "115200"))
HTTP_PORT = int(os.environ.get("ROBOT_HTTP_PORT", "8080"))

UART_CANDIDATES = (
    "/dev/serial0",
    "/dev/ttyAMA0",
    "/dev/ttyS0",
)

odom_lock = threading.Lock()
serial_lock = threading.Lock()

odom_state = {
    "x": 0.0,
    "y": 0.0,
    "theta_deg": 0.0,
    "rpm_a": 0.0,
    "rpm_b": 0.0,
    "pulse_a": 0,
    "pulse_b": 0,
    "speed_pct": 40.0,
    "joystick_k": 10.0,
    "v_max": 40.0,
    "last_esp_ms": 0,
}

ser = None


def send_uart(line: str) -> None:
    if ser is None or not ser.is_open:
        return
    payload = (line.strip() + "\n").encode("utf-8")
    with serial_lock:
        ser.write(payload)
        ser.flush()


def parse_esp_line(line: str) -> None:
    if not line.startswith("ODOM,"):
        return
    parts = line.split(",")
    if len(parts) < 8:
        return
    try:
        with odom_lock:
            odom_state["x"] = float(parts[1])
            odom_state["y"] = float(parts[2])
            odom_state["theta_deg"] = float(parts[3])
            odom_state["rpm_a"] = float(parts[4])
            odom_state["rpm_b"] = float(parts[5])
            odom_state["pulse_a"] = int(parts[6])
            odom_state["pulse_b"] = int(parts[7])
            odom_state["last_esp_ms"] = int(time.time() * 1000)
    except ValueError:
        pass


def uart_reader() -> None:
    buf = b""
    while True:
        try:
            if ser is None or not ser.is_open:
                time.sleep(0.2)
                continue
            chunk = ser.read(max(1, ser.in_waiting))
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    parse_esp_line(line)
        except serial.SerialException as exc:
            print(f"[uart] {exc}", flush=True)
            time.sleep(1.0)


def read_json_body(handler: BaseHTTPRequestHandler) -> dict:
    length = int(handler.headers.get("Content-Length", 0))
    if length <= 0:
        return {}
    raw = handler.rfile.read(length)
    try:
        return json.loads(raw.decode("utf-8"))
    except json.JSONDecodeError:
        return {}


def odom_json() -> str:
    with odom_lock:
        return json.dumps(odom_state)


class RobotHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        if args and str(args[0]).startswith("GET /odom"):
            return
        print(f"[http] {self.address_string()} {fmt % args}", flush=True)

    def _send(self, code: int, body: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, code: int, text: str) -> None:
        self._send(code, text.encode("utf-8"), "text/plain; charset=utf-8")

    def _send_json(self, code: int, obj) -> None:
        self._send(code, json.dumps(obj).encode("utf-8"), "application/json")

    def _serve_file(self, path: Path) -> None:
        if not path.is_file():
            self._send_text(404, "Not found")
            return
        suffix = path.suffix.lower()
        ctype = "text/html; charset=utf-8"
        if suffix == ".js":
            ctype = "application/javascript"
        elif suffix == ".css":
            ctype = "text/css"
        data = path.read_bytes()
        self._send(200, data, ctype)

    def do_GET(self) -> None:
        path = urlparse(self.path).path

        if path in ("/", "/control", "/control/", "/map", "/map/"):
            return self._serve_file(WEB_CONTROL / "index.html")
        if path == "/odom":
            return self._send(200, odom_json().encode("utf-8"), "application/json")

        self._send_text(404, "Not found")

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        data = read_json_body(self)

        if path == "/joystick":
            angle = float(data.get("angle", 0))
            mag = float(data.get("mag", 0))
            send_uart(f"JOY,{angle:.2f},{mag:.4f}")
            with odom_lock:
                odom_state["last_esp_ms"] = int(time.time() * 1000)
            return self._send_text(200, "OK")

        if path == "/stop":
            send_uart("STOP")
            return self._send_text(200, "OK")

        if path == "/speed":
            pct = float(data.get("pct", 40))
            pct = max(0.0, min(100.0, pct))
            send_uart(f"SPD,{pct:.0f}")
            with odom_lock:
                odom_state["speed_pct"] = pct
                odom_state["v_max"] = 100.0 * pct / 100.0
            return self._send_text(200, "OK")

        if path == "/k":
            k = float(data.get("k", 10))
            k = max(1.0, min(20.0, k))
            send_uart(f"K,{k:.0f}")
            with odom_lock:
                odom_state["joystick_k"] = k
            return self._send_text(200, "OK")

        if path == "/odom/reset":
            send_uart("ODOMRST")
            return self._send_text(200, "OK")

        self._send_text(404, "Not found")


def resolve_serial_port() -> Optional[str]:
    if SERIAL_PORT:
        return SERIAL_PORT if os.path.exists(SERIAL_PORT) else None
    for path in UART_CANDIDATES:
        if os.path.exists(path):
            return path
    return None


def uart_hint() -> None:
    print("[uart] WARN: No UART device yet.", file=sys.stderr, flush=True)
    print("  Tried:", ", ".join(UART_CANDIDATES), file=sys.stderr, flush=True)
    print("  Web UI still runs; UART will retry in background.", file=sys.stderr, flush=True)
    print("  Enable UART: sudo raspi-config -> Serial Port -> shell: No, UART: Yes", file=sys.stderr, flush=True)


def open_serial(port: str) -> serial.Serial:
    print(f"[uart] Opening {port} @ {BAUD}", flush=True)
    return serial.Serial(port, BAUD, timeout=0.05)


def uart_connect_loop() -> None:
    global ser
    warned = False
    while True:
        try:
            if ser is not None and ser.is_open:
                time.sleep(2.0)
                continue
            port = resolve_serial_port()
            if not port:
                if not warned:
                    uart_hint()
                    warned = True
                time.sleep(3.0)
                continue
            warned = False
            with serial_lock:
                if ser is not None:
                    try:
                        ser.close()
                    except serial.SerialException:
                        pass
                ser = open_serial(port)
            time.sleep(0.3)
            send_uart("SPD,40")
            send_uart("K,10")
            print("[uart] Connected", flush=True)
        except serial.SerialException as exc:
            print(f"[uart] Connect failed: {exc}", flush=True)
            with serial_lock:
                if ser is not None:
                    try:
                        ser.close()
                    except serial.SerialException:
                        pass
                    ser = None
            time.sleep(3.0)


def main() -> None:
    threading.Thread(target=uart_connect_loop, daemon=True).start()
    threading.Thread(target=uart_reader, daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), RobotHandler)
    print(f"[http] Robot UI: http://0.0.0.0:{HTTP_PORT}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if ser is not None and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
