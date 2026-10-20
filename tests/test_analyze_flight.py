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


def test_parse_truncated_and_empty_files(tmp_path):
    f = tmp_path / "trunc.bin"
    f.write_bytes(pack_record() + b"\x00" * 5)  # 1 full record + junk tail
    assert len(af.parse_log(str(f))) == 1
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
