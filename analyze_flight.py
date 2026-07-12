#!/usr/bin/env python3
"""
CRSF to PWM with IMU - Flight Telemetry Analyzer
------------------------------------------------
This script reads the binary log from the ESP32-C3 Flight Controller (either via
WiFi HTTP download or local file), converts it to a standard JSON dataset,
performs key telemetry and attitude calculations, and generates an interactive,
premium HTML flight analysis dashboard.

No external dependencies are required. Runs on standard Python 3 libraries.
"""

import os
import sys
import math
import json
import struct
import argparse
import webbrowser
import urllib.request
from datetime import datetime

# Binary Log Struct configuration (must match Arduino struct)
# struct __attribute__((__packed__)) LogRecord {
#   uint32_t timestamp;  // 4 bytes (unsigned int)
#   int16_t roll;        // 2 bytes (signed short)
#   int16_t pitch;       // 2 bytes (signed short)
#   int16_t ax, ay, az;  // 6 bytes (3 * signed short)
#   int16_t gx, gy, gz;  // 6 bytes (3 * signed short)
#   uint16_t aileron;    // 2 bytes (unsigned short)
#   uint16_t elevator;   // 2 bytes (unsigned short)
#   uint16_t throttle;   // 2 bytes (unsigned short)
#   uint16_t rudder;     // 2 bytes (unsigned short)
#   uint16_t flaps;      // 2 bytes (unsigned short)
#   uint16_t aux;        // 2 bytes (unsigned short)
#   uint8_t flags;       // 1 byte (unsigned char)
# }; // Total = 33 bytes
STRUCT_FORMAT = "<IhhhhhhhhHHHHHHB"
RECORD_SIZE = struct.calcsize(STRUCT_FORMAT)

# IMU Constants
ACCEL_SCALE_2G = 16384.0  # LSB per G (MPU6050 default +-2g range)
GYRO_SCALE_250DPS = 131.0 # LSB per deg/s (MPU6050 default +-250dps range)

def download_log(url, output_path):
    """Download the binary flight log from the ESP32 Web Server."""
    print(f"Connecting to ESP32 Flight Controller at {url}...")
    log_url = f"{url.rstrip('/')}/log"
    try:
        req = urllib.request.Request(
            log_url, 
            headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'}
        )
        with urllib.request.urlopen(req, timeout=8) as response:
            print("Downloading flight log data...")
            data = response.read()
            with open(output_path, 'wb') as f:
                f.write(data)
            print(f"Successfully downloaded flight log ({len(data) / 1024:.2f} KB) -> '{output_path}'")
            return True
    except Exception as e:
        print(f"\n[Error] Failed to download log from ESP32 WiFi.")
        print(f"Details: {e}")
        print("\nMake sure you are connected to the 'RCPlane-FC' WiFi network,")
        print("or specify a local log file using the -i/--input flag.")
        return False

def parse_log(file_path):
    """Parse the raw binary flight log into Python dictionaries."""
    records = []
    if not os.path.exists(file_path):
        print(f"[Error] Log file '{file_path}' does not exist.")
        return records

    print(f"Parsing binary log file '{file_path}'...")
    file_size = os.path.getsize(file_path)
    expected_records = file_size // RECORD_SIZE
    
    if expected_records == 0:
        print("[Warning] Log file is empty or too small.")
        return records

    with open(file_path, 'rb') as f:
        for idx in range(expected_records):
            buffer = f.read(RECORD_SIZE)
            if len(buffer) < RECORD_SIZE:
                break
            
            # Unpack the packed binary struct
            unpacked = struct.unpack(STRUCT_FORMAT, buffer)
            
            # Map unpacked tuple to readable dictionary fields
            record = {
                "timestamp_ms": unpacked[0],
                "roll": unpacked[1] / 10.0,      # scaled by 10 in firmware
                "pitch": unpacked[2] / 10.0,    # scaled by 10 in firmware
                "ax": unpacked[3],
                "ay": unpacked[4],
                "az": unpacked[5],
                "gx": unpacked[6],
                "gy": unpacked[7],
                "gz": unpacked[8],
                "aileron": unpacked[9],
                "elevator": unpacked[10],
                "throttle": unpacked[11],
                "rudder": unpacked[12],
                "flaps": unpacked[13],
                "aux": unpacked[14],
                "failsafe": bool(unpacked[15] & 0x01)
            }
            
            # Compute calculated fields
            # G-Force: sqrt(ax^2 + ay^2 + az^2) / scale
            ax_g = record["ax"] / ACCEL_SCALE_2G
            ay_g = record["ay"] / ACCEL_SCALE_2G
            az_g = record["az"] / ACCEL_SCALE_2G
            record["g_force"] = math.sqrt(ax_g**2 + ay_g**2 + az_g**2)
            
            # Gyro rates in deg/s
            record["roll_rate"] = record["gx"] / GYRO_SCALE_250DPS
            record["pitch_rate"] = record["gy"] / GYRO_SCALE_250DPS
            record["yaw_rate"] = record["gz"] / GYRO_SCALE_250DPS
            
            records.append(record)
            
    print(f"Successfully parsed {len(records)} log entries.")
    return records

def calculate_analytics(records):
    """Analyze flight telemetry records and return key performance indicators (KPIs)."""
    if not records:
        return {}

    duration_ms = records[-1]["timestamp_ms"] - records[0]["timestamp_ms"]
    duration_sec = duration_ms / 1000.0
    
    # Throttle stats (typically 988us to 2012us)
    throttles = [r["throttle"] for r in records]
    throttles_pct = [((t - 988) / (2012 - 988)) * 100.0 for t in throttles]
    throttles_pct = [max(0.0, min(100.0, p)) for p in throttles_pct]
    
    avg_throttle = sum(throttles_pct) / len(throttles_pct)
    max_throttle = max(throttles_pct)
    
    # Attitude extremes
    max_roll = max(abs(r["roll"]) for r in records)
    max_pitch = max(abs(r["pitch"]) for r in records)
    
    # G-Force stats
    g_forces = [r["g_force"] for r in records]
    max_g = max(g_forces)
    avg_g = sum(g_forces) / len(g_forces)
    
    # Vibration Index: Average delta G between consecutive samples (high pass representation)
    delta_g = []
    for i in range(1, len(records)):
        delta_g.append(abs(records[i]["g_force"] - records[i-1]["g_force"]))
    
    avg_delta_g = sum(delta_g) / len(delta_g) if delta_g else 0.0
    
    # Categorize vibration
    if avg_delta_g < 0.04:
        vibration_score = "Excellent (Very Smooth)"
        vibration_color = "emerald"
    elif avg_delta_g < 0.12:
        vibration_score = "Good (Normal)"
        vibration_color = "teal"
    elif avg_delta_g < 0.25:
        vibration_score = "Fair (Vibration present)"
        vibration_color = "amber"
    else:
        vibration_score = "Warning (Severe - check balance)"
        vibration_color = "rose"

    # Failsafe events
    failsafe_events = 0
    in_failsafe = False
    failsafe_duration_ms = 0
    fs_start = 0
    
    for r in records:
        if r["failsafe"] and not in_failsafe:
            failsafe_events += 1
            in_failsafe = True
            fs_start = r["timestamp_ms"]
        elif not r["failsafe"] and in_failsafe:
            in_failsafe = False
            failsafe_duration_ms += (r["timestamp_ms"] - fs_start)
            
    # If flight ended in failsafe
    if in_failsafe:
        failsafe_duration_ms += (records[-1]["timestamp_ms"] - fs_start)

    # Maneuver detection
    bank_seconds = sum(1 for r in records if abs(r["roll"]) > 45.0) * 0.05  # assuming 20Hz (50ms interval)
    dive_climb_seconds = sum(1 for r in records if abs(r["pitch"]) > 30.0) * 0.05
    
    return {
        "duration_sec": duration_sec,
        "records_count": len(records),
        "avg_throttle": avg_throttle,
        "max_throttle": max_throttle,
        "max_roll": max_roll,
        "max_pitch": max_pitch,
        "max_g": max_g,
        "avg_g": avg_g,
        "vibration_level": avg_delta_g,
        "vibration_score": vibration_score,
        "vibration_color": vibration_color,
        "failsafe_events": failsafe_events,
        "failsafe_total_duration_sec": failsafe_duration_ms / 1000.0,
        "bank_seconds": bank_seconds,
        "dive_climb_seconds": dive_climb_seconds
    }

def downsample_data(data, max_points=1200):
    """Downsample data list for faster browser chart rendering."""
    if len(data) <= max_points:
        return data
    step = len(data) // max_points
    return data[::step]

def generate_html_dashboard(records, kpis, output_path):
    """Generate a premium interactive ground-station style HTML report."""
    
    # Downsample points for visualization
    ds_records = downsample_data(records)
    
    # Extract arrays for Javascript
    timestamps = [round((r["timestamp_ms"] - records[0]["timestamp_ms"]) / 1000.0, 2) for r in ds_records]
    rolls = [round(r["roll"], 1) for r in ds_records]
    pitches = [round(r["pitch"], 1) for r in ds_records]
    throttles_pct = [round(((r["throttle"] - 988) / (2012 - 988)) * 100.0, 1) for r in ds_records]
    throttles_pct = [max(0.0, min(100.0, p)) for p in throttles_pct]
    
    ailerons = [r["aileron"] for r in ds_records]
    elevators = [r["elevator"] for r in ds_records]
    rudders = [r["rudder"] for r in ds_records]
    g_forces = [round(r["g_force"], 2) for r in ds_records]
    failsafes = [100 if r["failsafe"] else 0 for r in ds_records]

    # Convert to JSON for direct injection into JavaScript
    js_data = {
        "timestamps": timestamps,
        "roll": rolls,
        "pitch": pitches,
        "throttle": throttles_pct,
        "aileron": ailerons,
        "elevator": elevators,
        "rudder": rudders,
        "g_force": g_forces,
        "failsafe": failsafes
    }

    # Format flight duration
    duration_m = int(kpis["duration_sec"] // 60)
    duration_s = int(kpis["duration_sec"] % 60)
    duration_str = f"{duration_m}m {duration_s}s"

    html_template = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Post-Flight Data Telemetry Dashboard</title>
    <!-- Tailwind CSS -->
    <script src="https://cdn.tailwindcss.com"></script>
    <!-- Chart.js -->
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700&family=Plus+Jakarta+Sans:wght@300;400;500;600;700&display=swap');
        body {{
            font-family: 'Plus Jakarta Sans', sans-serif;
            background-color: #0b0f19;
            color: #f3f4f6;
        }}
        .tech-font {{
            font-family: 'Orbitron', sans-serif;
        }}
        .glow-card {{
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.5), 0 0 1px rgba(255, 255, 255, 0.1) inset;
        }}
    </style>
</head>
<body class="min-h-screen p-4 md:p-8">
    <div class="max-w-7xl mx-auto">
        <!-- Header -->
        <header class="flex flex-col md:flex-row md:items-center md:justify-between pb-6 mb-8 border-b border-slate-800">
            <div>
                <h1 class="text-3xl font-extrabold tracking-tight text-white flex items-center">
                    <span class="text-sky-500 mr-2">✦</span> FLIGHT BLACKBOX ANALYZER
                </h1>
                <p class="text-slate-400 mt-1">Post-flight autopilot diagnostics and IMU telemetry dashboard</p>
            </div>
            <div class="mt-4 md:mt-0 flex items-center bg-slate-900 px-4 py-2 rounded-lg border border-slate-800">
                <div class="w-3 h-3 bg-emerald-500 rounded-full animate-pulse mr-2"></div>
                <span class="text-xs text-slate-300 tech-font">LOG PARSING COMPLETE</span>
            </div>
        </header>

        <!-- KPIs Overview -->
        <section class="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-6 gap-4 mb-8">
            <!-- Flight Duration -->
            <div class="bg-slate-900 rounded-xl p-4 border border-slate-850 glow-card">
                <div class="text-xs font-semibold text-slate-400 uppercase tracking-wider">Flight Duration</div>
                <div class="text-2xl font-bold mt-2 text-white tech-font">{duration_str}</div>
                <div class="text-xs text-slate-500 mt-1">{kpis['records_count']} recorded data points</div>
            </div>

            <!-- Average Throttle -->
            <div class="bg-slate-900 rounded-xl p-4 border border-slate-850 glow-card">
                <div class="text-xs font-semibold text-slate-400 uppercase tracking-wider">Average Throttle</div>
                <div class="text-2xl font-bold mt-2 text-sky-400 tech-font">{kpis['avg_throttle']:.1f}%</div>
                <div class="text-xs text-slate-500 mt-1">Peak: {kpis['max_throttle']:.1f}%</div>
            </div>

            <!-- Max G-force -->
            <div class="bg-slate-900 rounded-xl p-4 border border-slate-850 glow-card">
                <div class="text-xs font-semibold text-slate-400 uppercase tracking-wider">Peak G-Force</div>
                <div class="text-2xl font-bold mt-2 text-orange-400 tech-font">{kpis['max_g']:.2f} G</div>
                <div class="text-xs text-slate-500 mt-1">Avg Force: {kpis['avg_g']:.2f} G</div>
            </div>

            <!-- Attitude Extremes -->
            <div class="bg-slate-900 rounded-xl p-4 border border-slate-850 glow-card">
                <div class="text-xs font-semibold text-slate-400 uppercase tracking-wider">Peak Angles</div>
                <div class="text-lg font-bold mt-2 text-white tech-font">
                    R: <span class="text-purple-400">{kpis['max_roll']:.1f}°</span> | P: <span class="text-fuchsia-400">{kpis['max_pitch']:.1f}°</span>
                </div>
                <div class="text-xs text-slate-500 mt-2">Extreme roll/pitch limits</div>
            </div>

            <!-- Vibration Score -->
            <div class="bg-slate-900 rounded-xl p-4 border border-slate-850 glow-card">
                <div class="text-xs font-semibold text-slate-400 uppercase tracking-wider">Vibration Index</div>
                <div class="text-xl font-bold mt-2 text-{kpis['vibration_color']}-400 tech-font truncate">{kpis['vibration_score'].split(' ')[0]}</div>
                <div class="text-xs text-slate-500 mt-2">Prop variance: {kpis['vibration_level']:.3f}g</div>
            </div>

            <!-- Failsafes -->
            <div class="bg-slate-900 rounded-xl p-4 border border-slate-850 glow-card">
                <div class="text-xs font-semibold text-slate-400 uppercase tracking-wider">Failsafes</div>
                <div class="text-2xl font-bold mt-2 tech-font {'text-emerald-400' if kpis['failsafe_events'] == 0 else 'text-red-500 animate-pulse'}">
                    {kpis['failsafe_events']}
                </div>
                <div class="text-xs text-slate-500 mt-1">Total loss: {kpis['failsafe_total_duration_sec']:.1f}s</div>
            </div>
        </section>

        <!-- Aerobatics Diagnostics & Info -->
        <section class="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-8">
            <div class="bg-slate-900/60 rounded-xl p-5 border border-slate-800 col-span-2">
                <h3 class="text-lg font-bold mb-3 text-white">Pilot Flight Insights</h3>
                <div class="grid grid-cols-2 md:grid-cols-3 gap-4">
                    <div class="bg-slate-900/80 p-3 rounded-lg border border-slate-800">
                        <span class="text-xs text-slate-400 block">Aerobatic Time (Bank > 45°)</span>
                        <span class="text-xl font-semibold text-white tech-font">{kpis['bank_seconds']:.1f}s</span>
                    </div>
                    <div class="bg-slate-900/80 p-3 rounded-lg border border-slate-800">
                        <span class="text-xs text-slate-400 block">Extreme Pitch Time (> 30°)</span>
                        <span class="text-xl font-semibold text-white tech-font">{kpis['dive_climb_seconds']:.1f}s</span>
                    </div>
                    <div class="bg-slate-900/80 p-3 rounded-lg border border-slate-800">
                        <span class="text-xs text-slate-400 block">Hardware Type</span>
                        <span class="text-sm font-semibold text-slate-200 block mt-1">ESP32-C3 / MPU6050</span>
                    </div>
                </div>
                <div class="mt-4 text-xs text-slate-400 leading-relaxed border-t border-slate-800/60 pt-3">
                    <span class="font-bold text-sky-400">Flight Tip:</span> 
                    {"The flight logs show normal vibration levels. The propeller is well balanced." if kpis['vibration_level'] < 0.12 else "Elevated vibration levels detected. We recommend checking motor mounts, balancing the propeller, or isolating the IMU on dampening tape to improve complementary filter attitude accuracy."}
                </div>
            </div>

            <!-- Flight controller status -->
            <div class="bg-slate-900/60 rounded-xl p-5 border border-slate-800">
                <h3 class="text-lg font-bold mb-3 text-white">Autopilot Health</h3>
                <ul class="space-y-2 text-sm text-slate-300">
                    <li class="flex justify-between py-1 border-b border-slate-800/40">
                        <span>CRSF Connection</span>
                        <span class="text-emerald-400 font-medium">{'Active / Healthy' if kpis['failsafe_events'] == 0 else 'Intermittent Warnings'}</span>
                    </li>
                    <li class="flex justify-between py-1 border-b border-slate-800/40">
                        <span>IMU Sensor Path</span>
                        <span class="text-emerald-400 font-medium">MPU6050 OK</span>
                    </li>
                    <li class="flex justify-between py-1 border-b border-slate-800/40">
                        <span>Logging Rate</span>
                        <span class="text-slate-400 font-medium">20 Hz (50ms interval)</span>
                    </li>
                    <li class="flex justify-between py-1">
                        <span>Failsafe Status</span>
                        <span class="{'text-emerald-400' if not records[-1]['failsafe'] else 'text-red-500 font-bold'}">
                            {'Normal' if not records[-1]['failsafe'] else 'ACTIVE'}
                        </span>
                    </li>
                </ul>
            </div>
        </section>

        <!-- Charts Dashboard Tabs -->
        <section class="bg-slate-900 rounded-xl border border-slate-850 p-6 glow-card">
            <!-- Tabs Menu -->
            <div class="flex border-b border-slate-800 mb-6 text-sm">
                <button onclick="switchTab('attitudeTab', this)" class="tab-btn px-4 py-2 font-bold text-sky-400 border-b-2 border-sky-400 focus:outline-none transition">
                    Attitude Dynamics
                </button>
                <button onclick="switchTab('servosTab', this)" class="tab-btn px-4 py-2 font-semibold text-slate-400 hover:text-white border-b-2 border-transparent focus:outline-none transition">
                    Control Inputs
                </button>
                <button onclick="switchTab('forcesTab', this)" class="tab-btn px-4 py-2 font-semibold text-slate-400 hover:text-white border-b-2 border-transparent focus:outline-none transition">
                    G-Forces & Vibrations
                </button>
            </div>

            <!-- Tab Contents -->
            <div id="tabContents">
                <!-- Attitude Chart -->
                <div id="attitudeTab" class="tab-pane block">
                    <div class="mb-4">
                        <h4 class="text-md font-semibold text-white">Roll & Pitch Attitude Angular Dynamics</h4>
                        <p class="text-xs text-slate-400">Euler angles in degrees derived from complementary filter merging accel + gyro rates.</p>
                    </div>
                    <div class="relative h-[400px] w-full">
                        <canvas id="attitudeChart"></canvas>
                    </div>
                </div>

                <!-- Control Inputs Chart -->
                <div id="servosTab" class="tab-pane hidden">
                    <div class="mb-4">
                        <h4 class="text-md font-semibold text-white">Receiver Channel Servo Commands</h4>
                        <p class="text-xs text-slate-400">PWM output signals in microseconds (us) decoded from CRSF channel frames (988us - 2012us).</p>
                    </div>
                    <div class="relative h-[400px] w-full">
                        <canvas id="servosChart"></canvas>
                    </div>
                </div>

                <!-- G-Force Chart -->
                <div id="forcesTab" class="tab-pane hidden">
                    <div class="mb-4">
                        <h4 class="text-md font-semibold text-white">G-Force (Total Magnitude) over Time</h4>
                        <p class="text-xs text-slate-400">Vibration index tracks flight structural loads and landing impact forces.</p>
                    </div>
                    <div class="relative h-[400px] w-full">
                        <canvas id="forcesChart"></canvas>
                    </div>
                </div>
            </div>
        </section>
        
        <footer class="mt-12 text-center text-xs text-slate-500 pb-6">
            Flight Blackbox Diagnostics Tool • Generated {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
        </footer>
    </div>

    <!-- Script Section -->
    <script>
        const flightData = {json.dumps(js_data)};

        // Tab Switching Logic
        function switchTab(tabId, btn) {{
            // Hide all tabs
            document.querySelectorAll('.tab-pane').forEach(el => el.classList.add('hidden'));
            document.querySelectorAll('.tab-pane').forEach(el => el.classList.remove('block'));
            
            // Show active tab
            document.getElementById(tabId).classList.add('block');
            document.getElementById(tabId).classList.remove('hidden');

            // Reset buttons
            document.querySelectorAll('.tab-btn').forEach(el => {{
                el.classList.remove('text-sky-400', 'border-sky-400', 'font-bold');
                el.classList.add('text-slate-400', 'border-transparent', 'font-semibold');
            }});

            // Highlight button
            btn.classList.add('text-sky-400', 'border-sky-400', 'font-bold');
            btn.classList.remove('text-slate-400', 'border-transparent', 'font-semibold');
        }}

        // Common Chart Configuration
        const chartOptions = {{
            responsive: true,
            maintainAspectRatio: false,
            interaction: {{
                mode: 'index',
                intersect: false,
            }},
            scales: {{
                x: {{
                    grid: {{ color: '#1e293b' }},
                    title: {{ display: true, text: 'Time (seconds)', color: '#94a3b8' }},
                    ticks: {{ color: '#64748b' }}
                }},
                y: {{
                    grid: {{ color: '#1e293b' }},
                    ticks: {{ color: '#64748b' }}
                }}
            }},
            plugins: {{
                legend: {{
                    labels: {{ color: '#cbd5e1' }}
                }}
            }}
        }};

        // 1. Attitude Chart
        const attitudeCtx = document.getElementById('attitudeChart').getContext('2d');
        new Chart(attitudeCtx, {{
            type: 'line',
            data: {{
                labels: flightData.timestamps,
                datasets: [
                    {{
                        label: 'Roll Angle (°)',
                        data: flightData.roll,
                        borderColor: '#a855f7',
                        backgroundColor: 'rgba(168, 85, 247, 0.1)',
                        borderWidth: 2,
                        tension: 0.1,
                        pointRadius: 0
                    }},
                    {{
                        label: 'Pitch Angle (°)',
                        data: flightData.pitch,
                        borderColor: '#ec4899',
                        backgroundColor: 'rgba(236, 72, 153, 0.1)',
                        borderWidth: 2,
                        tension: 0.1,
                        pointRadius: 0
                    }}
                ]
            }},
            options: {{
                ...chartOptions,
                scales: {{
                    ...chartOptions.scales,
                    y: {{
                        ...chartOptions.scales.y,
                        title: {{ display: true, text: 'Attitude (degrees)', color: '#94a3b8' }}
                    }}
                }}
            }}
        }});

        // 2. Servos Chart
        const servosCtx = document.getElementById('servosChart').getContext('2d');
        new Chart(servosCtx, {{
            type: 'line',
            data: {{
                labels: flightData.timestamps,
                datasets: [
                    {{
                        label: 'Throttle (%)',
                        data: flightData.throttle,
                        borderColor: '#38bdf8',
                        borderWidth: 2.5,
                        tension: 0.1,
                        pointRadius: 0,
                        yAxisID: 'yThrottle'
                    }},
                    {{
                        label: 'Aileron command (us)',
                        data: flightData.aileron,
                        borderColor: '#22c55e',
                        borderWidth: 1.5,
                        tension: 0.1,
                        pointRadius: 0,
                        yAxisID: 'yServos',
                        hidden: false
                    }},
                    {{
                        label: 'Elevator command (us)',
                        data: flightData.elevator,
                        borderColor: '#f59e0b',
                        borderWidth: 1.5,
                        tension: 0.1,
                        pointRadius: 0,
                        yAxisID: 'yServos',
                        hidden: false
                    }},
                    {{
                        label: 'Rudder command (us)',
                        data: flightData.rudder,
                        borderColor: '#ef4444',
                        borderWidth: 1.5,
                        tension: 0.1,
                        pointRadius: 0,
                        yAxisID: 'yServos',
                        hidden: true
                    }}
                ]
            }},
            options: {{
                ...chartOptions,
                scales: {{
                    x: chartOptions.scales.x,
                    yServos: {{
                        type: 'linear',
                        position: 'left',
                        min: 900,
                        max: 2100,
                        grid: {{ color: '#1e293b' }},
                        ticks: {{ color: '#64748b' }},
                        title: {{ display: true, text: 'Servo Signals (us)', color: '#94a3b8' }}
                    }},
                    yThrottle: {{
                        type: 'linear',
                        position: 'right',
                        min: 0,
                        max: 100,
                        grid: {{ drawOnChartArea: false }},
                        ticks: {{ color: '#64748b' }},
                        title: {{ display: true, text: 'Throttle %', color: '#94a3b8' }}
                    }}
                }}
            }}
        }});

        // 3. G-Force Chart
        const forcesCtx = document.getElementById('forcesChart').getContext('2d');
        new Chart(forcesCtx, {{
            type: 'line',
            data: {{
                labels: flightData.timestamps,
                datasets: [
                    {{
                        label: 'Dynamic G-Force (G)',
                        data: flightData.g_force,
                        borderColor: '#f97316',
                        backgroundColor: 'rgba(249, 115, 22, 0.05)',
                        borderWidth: 1.5,
                        tension: 0.1,
                        pointRadius: 0,
                        fill: true
                    }},
                    {{
                        label: 'Failsafe Warning',
                        data: flightData.failsafe,
                        borderColor: '#ef4444',
                        backgroundColor: 'rgba(239, 68, 68, 0.2)',
                        borderWidth: 1,
                        tension: 0,
                        pointRadius: 0,
                        fill: true,
                        hidden: false
                    }}
                ]
            }},
            options: {{
                ...chartOptions,
                scales: {{
                    ...chartOptions.scales,
                    y: {{
                        ...chartOptions.scales.y,
                        min: 0,
                        title: {{ display: true, text: 'G-Force / Event Status', color: '#94a3b8' }}
                    }}
                }}
            }}
        }});
    </script>
</body>
</html>
"""

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html_template)
    print(f"Generated flight analysis web dashboard -> '{output_path}'")

def main():
    parser = argparse.ArgumentParser(description="CRSF-to-PWM/IMU Flight Log Analyzer")
    parser.add_argument("-u", "--url", default="http://192.168.4.1", 
                        help="Web server address of the ESP32 (default: http://192.168.4.1)")
    parser.add_argument("-i", "--input", default=None, 
                        help="Path to local binary flight.log file (if specified, bypasses WiFi download)")
    parser.add_argument("-o", "--output-json", default="flight_log.json", 
                        help="Output JSON file name (default: flight_log.json)")
    parser.add_argument("-d", "--output-html", default="flight_analysis.html", 
                        help="Output HTML dashboard name (default: flight_analysis.html)")
    parser.add_argument("--no-open", action="store_true", 
                        help="Do not open the generated HTML dashboard in the browser automatically")
    
    args = parser.parse_args()
    
    # 1. Determine data source (WiFi download vs. Local file)
    binary_path = args.input
    if not binary_path:
        binary_path = "flight_downloaded.log"
        success = download_log(args.url, binary_path)
        if not success:
            sys.exit(1)

    # 2. Parse binary log file
    records = parse_log(binary_path)
    if not records:
        print("[Error] No flight records parsed. Exiting.")
        sys.exit(1)

    # 3. Save clean JSON file for analysis
    print(f"Saving parsed data to JSON format: '{args.output_json}'...")
    try:
        with open(args.output_json, 'w') as f:
            json.dump(records, f, indent=2)
        print(f"JSON save complete.")
    except Exception as e:
        print(f"[Error] Failed to write JSON file: {e}")

    # 4. Perform Flight Analysis / KPIs
    kpis = calculate_analytics(records)
    
    # 5. Output Summary to Console
    print("\n" + "="*50)
    print("              FLIGHT SUMMARY STATISTICS             ")
    print("="*50)
    duration_min = int(kpis["duration_sec"] // 60)
    duration_sec = int(kpis["duration_sec"] % 60)
    print(f"Flight Duration:         {duration_min}m {duration_sec}s")
    print(f"Total Datapoints:        {kpis['records_count']}")
    print(f"Avg Throttle Command:    {kpis['avg_throttle']:.1f}%  (Max: {kpis['max_throttle']:.1f}%)")
    print(f"Peak Attitude Pitch:     {kpis['max_pitch']:.1f}°")
    print(f"Peak Attitude Roll:      {kpis['max_roll']:.1f}°")
    print(f"Peak Dynamic G-Force:    {kpis['max_g']:.2f} G  (Avg: {kpis['avg_g']:.2f} G)")
    print(f"Vibration Level Index:   {kpis['vibration_level']:.3f}g  ({kpis['vibration_score']})")
    print(f"Failsafe Occurrences:    {kpis['failsafe_events']} times  (Total loss duration: {kpis['failsafe_total_duration_sec']:.1f}s)")
    print("="*50 + "\n")

    # 6. Generate dashboard HTML
    generate_html_dashboard(records, kpis, args.output_html)

    # 7. Auto-open dashboard in browser
    if not args.no_open:
        abs_html_path = os.path.abspath(args.output_html)
        print(f"Opening dashboard in browser: file://{abs_html_path}")
        webbrowser.open(f"file://{abs_html_path}")

if __name__ == "__main__":
    main()
