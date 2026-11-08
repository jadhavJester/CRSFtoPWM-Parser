"""Tests for analyze_flight.py — the ESP32-C3 flight-log analyzer.

The binary record format must stay in lockstep with the packed LogRecord
struct in the Arduino firmware (33 bytes, little-endian). These tests pin
that contract, the unit scaling, and the analytics KPIs, using both
synthetic records and the mock_flight.log fixture shipped in the repo.
"""

import struct
from pathlib import Path

import pytest

import analyze_flight as af

REPO = Path(__file__).resolve().parents[1]
MOCK_LOG = REPO / "mock_flight.log"


def pack_record(ts=1000, roll=0, pitch=0, ax=0, ay=0, az=0,
                gx=0, gy=0, gz=0, aileron=1500, elevator=1500,
                throttle=1000, rudder=1500, flaps=988, aux=988, flags=0):
    return struct.pack(af.STRUCT_FORMAT, ts, roll, pitch, ax, ay, az,
                       gx, gy, gz, aileron, elevator, throttle, rudder,
                       flaps, aux, flags)


def test_record_format_matches_firmware_struct():
    assert af.RECORD_SIZE == 33
    assert af.RECORD_SIZE == struct.calcsize("<IhhhhhhhhHHHHHHB")


def test_parse_synthetic_record_scaling(tmp_path):
    # roll raw is decidegrees, accel raw is LSB at 16384 per g,
    # gyro raw is LSB at 131 per deg/s, flags bit0 is failsafe.
    blob = pack_record(ts=4200, roll=123, pitch=-45, ax=16384, gx=131,
                       throttle=2012, flags=0x01)
    f = tmp_path / "one.bin"
    f.write_bytes(blob)
    (rec,) = af.parse_log(str(f))
    assert rec["timestamp_ms"] == 4200
    assert rec["roll"] == pytest.approx(12.3)
    assert rec["pitch"] == pytest.approx(-4.5)
    assert rec["g_force"] == pytest.approx(1.0)
    assert rec["roll_rate"] == pytest.approx(1.0)
    assert rec["throttle"] == 2012
    assert rec["failsafe"] is True


def test_parse_truncated_and_empty_files(tmp_path, capsys):
    f = tmp_path / "trunc.bin"
    f.write_bytes(pack_record() + b"\x00" * 5)  # 1 full record + junk tail
    assert len(af.parse_log(str(f))) == 1
    assert "trailing" in capsys.readouterr().out
    empty = tmp_path / "empty.bin"
    empty.write_bytes(b"")
    assert af.parse_log(str(empty)) == []
    assert af.parse_log(str(tmp_path / "missing.bin")) == []


def test_parse_mock_flight_log():
    if not MOCK_LOG.exists():
        pytest.skip("mock_flight.log not present")
    records = af.parse_log(str(MOCK_LOG))
    assert len(records) == MOCK_LOG.stat().st_size // af.RECORD_SIZE == 2400
    timestamps = [r["timestamp_ms"] for r in records]
    assert timestamps == sorted(timestamps)
    assert all(r["g_force"] >= 0 for r in records)


def test_analytics_on_mock_log():
    if not MOCK_LOG.exists():
        pytest.skip("mock_flight.log not present")
    records = af.parse_log(str(MOCK_LOG))
    kpi = af.calculate_analytics(records)
    assert kpi["records_count"] == 2400
    assert kpi["duration_sec"] > 0
    assert 0.0 <= kpi["avg_throttle"] <= 100.0
    assert kpi["max_throttle"] >= kpi["avg_throttle"]
    assert kpi["max_g"] >= kpi["avg_g"] > 0
    assert kpi["failsafe_events"] >= 0


def test_analytics_empty_input():
    assert af.calculate_analytics([]) == {}


def test_downsample_behaviour():
    data = list(range(2400))
    assert af.downsample_data(data, max_points=2400) == data
    out = af.downsample_data(data, max_points=1200)
    assert len(out) == 1200 and out[0] == 0 and out[1] == 2


def test_downsample_never_exceeds_cap():
    # A list only slightly over the cap must still shrink: with a
    # floor-division step of 1 it used to pass through unchanged.
    out = af.downsample_data(list(range(1201)), max_points=1200)
    assert len(out) <= 1200 and out[0] == 0
    out = af.downsample_data(list(range(2401)), max_points=1200)
    assert len(out) <= 1200 and out[0] == 0


def test_maneuver_time_uses_measured_sample_interval():
    # Maneuver-time estimates must derive the log interval from the
    # timestamps, not assume the firmware's 20 Hz logging rate.
    recs = [
        {"timestamp_ms": i * 100, "roll": 60.0, "pitch": 0.0, "g_force": 1.0,
         "throttle": 1500, "failsafe": False}
        for i in range(10)
    ]
    kpi = af.calculate_analytics(recs)
    assert kpi["sample_interval_ms"] == pytest.approx(100.0)
    assert kpi["logging_rate_hz"] == pytest.approx(10.0)
    assert kpi["bank_seconds"] == pytest.approx(10 * 0.1)
    assert kpi["dive_climb_seconds"] == pytest.approx(0.0)


def test_maneuver_time_falls_back_for_single_record():
    recs = [{"timestamp_ms": 0, "roll": 60.0, "pitch": 0.0, "g_force": 1.0,
             "throttle": 1500, "failsafe": False}]
    kpi = af.calculate_analytics(recs)
    assert kpi["sample_interval_ms"] == pytest.approx(50.0)
    assert kpi["bank_seconds"] == pytest.approx(0.05)


def test_download_log_round_trip(tmp_path):
    # download_log is the only untested path in the analyzer: serve the
    # mock log from a local stdlib HTTP server and check the bytes land
    # intact, plus that an unreachable ESP32 fails cleanly (False, no raise).
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    payload = MOCK_LOG.read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/log":
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        out = tmp_path / "dl.log"
        assert af.download_log(f"http://127.0.0.1:{server.server_port}",
                               str(out)) is True
        assert out.read_bytes() == payload
    finally:
        server.shutdown()
        server.server_close()

    assert af.download_log("http://127.0.0.1:1", str(tmp_path / "x.log")) is False


def test_main_end_to_end_with_mock_log(tmp_path, monkeypatch):
    # Exercise the whole CLI path: -i mock log -> JSON + HTML dashboard.
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr("sys.argv", ["analyze_flight.py", "-i", str(MOCK_LOG),
                                     "-o", "out.json", "-d", "out.html",
                                     "--no-open"])
    af.main()
    import json as _json
    records = _json.loads((tmp_path / "out.json").read_text())
    assert len(records) == 2400
    assert records[0]["timestamp_ms"] == 0
    assert records[-1]["timestamp_ms"] == 119950
    html = (tmp_path / "out.html").read_text(encoding="utf-8")
    assert "FLIGHT BLACKBOX ANALYZER" in html
    assert "2400 recorded data points" in html


def test_dashboard_failsafe_uses_dedicated_axis(tmp_path):
    # Regression: failsafe markers must stay off the G-force y-axis. They
    # used to be plotted as 100 on the shared axis, which stretched the
    # scale and crushed the G trace whenever a failsafe occurred.
    import json as _json
    import re as _re

    recs = [
        {"timestamp_ms": i * 50, "roll": 0.0, "pitch": 0.0, "g_force": 1.0,
         "throttle": 1500, "aileron": 1500, "elevator": 1500,
         "rudder": 1500, "failsafe": (i == 3)}
        for i in range(6)
    ]
    kpi = af.calculate_analytics(recs)
    out = tmp_path / "dash.html"
    af.generate_html_dashboard(recs, kpi, str(out))
    html = out.read_text(encoding="utf-8")
    assert "yFailsafe" in html
    m = _re.search(r"const flightData = (\{.*?\});", html, _re.S)
    data = _json.loads(m.group(1))
    assert sorted(set(data["failsafe"])) <= [0, 1]
    assert data["failsafe"][3] == 1
    assert max(data["g_force"]) == pytest.approx(1.0)
