
# Autonomous Drone Surveillance System
### Ground Control Station — Flask + SQLite Backend

![Status](https://img.shields.io/badge/STATUS-CONCEPTUAL%20DESIGN-00d4ff?style=for-the-badge&labelColor=0d1828)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Flask](https://img.shields.io/badge/Flask-3.x-000000?style=for-the-badge&logo=flask&logoColor=white)
![SQLite](https://img.shields.io/badge/SQLite-Database-003B57?style=for-the-badge&logo=sqlite&logoColor=white)
![License](https://img.shields.io/badge/LICENSE-MIT-ffaa00?style=for-the-badge&labelColor=0d1828)

**Atılım University ·Multidisciplinary Design in Engineering · MECE 322 · Spring 2025–2026**

</div>

---

##  Overview

The **Autonomous Drone Surveillance System (ADSS)** is a fully conceptual, multidisciplinary engineering design project. This repository contains the complete **Ground Control Station (GCS)** web application — a real-time dashboard built with Flask and SQLite that simulates live drone telemetry, threat detection logging, acoustic monitoring, and fleet command-and-control.

> ⚠️ **This is a conceptual/simulation project.** No physical hardware is required. All telemetry is simulated server-side and persisted to a local SQLite database.

---

##  Features

| Feature | Description |
|---|---|
|  **Fleet Dashboard** | Live drone positions, battery levels, sector info, patrol map |
|  **Live Video Feed** | Simulated FLIR thermal feed with AI bounding boxes (HUMAN / VEHICLE) |
|  **Alert Log** | Full event history with acoustic TDOA spectrum monitor |
|  **System Status** | Module health (M1–M7), emergency controls, session summary |
|  **Database View** | Browse all 5 SQLite tables live — telemetry, events, detections |
|  **Live Simulation** | Server-side telemetry updates every API poll (battery drain, position drift) |
|  **REST API** | 10 endpoints for drones, events, detections, telemetry, session stats |
|  **Persistent Storage** | All data written to `adss.db` (SQLite) — survives restarts |

---

##  Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/yourusername/adss.git
cd adss
```

### 2. Install dependencies

```bash
pip install -r requirements.txt
```

### 3. Run the server

```bash
python app.py
```

### 4. Open in browser

```
http://127.0.0.1:5000
```

The database (`adss.db`) is created automatically on first run with seed data for 3 drones and 5 sample events.

---

## 📁 Project Structure

```
adss/
│
├── app.py                  ← Flask application + SQLite logic + REST API
├── requirements.txt        ← Python dependencies
├── README.md               ← This file
├── adss.db                 ← SQLite database (auto-created)
│
└── templates/
    └── index.html          ← Full GCS frontend (5 tabs, live API integration)
```

---

##  Database Schema

The application uses a single SQLite file (`adss.db`) with 5 tables:

```sql
┌─────────────────────────────────────────────────────────┐
│  drones          — fleet registry + live telemetry       │
│  events          — chronological alert & event log       │
│  detections      — AI object detection records           │
│  telemetry       — time-series snapshots (auto-logged)   │
│  session_stats   — cumulative mission statistics         │
└─────────────────────────────────────────────────────────┘
```

### `drones`

| Column | Type | Description |
|---|---|---|
| `id` | INTEGER PK | Drone identifier |
| `name` | TEXT | e.g. "DRONE 01" |
| `status` | TEXT | FLYING / DOCKING / CHARGING / STANDBY |
| `battery` | REAL | State of charge (%) |
| `altitude` | REAL | Current altitude (m) |
| `latitude` | REAL | GPS latitude |
| `longitude` | REAL | GPS longitude |
| `speed` | REAL | Ground speed (m/s) |
| `heading` | REAL | Heading (degrees) |
| `signal` | TEXT | Strong / Fair / Weak |
| `latency_ms` | INTEGER | RF link latency |
| `sector` | TEXT | Patrol sector label |
| `flight_time` | INTEGER | Cumulative flight seconds |
| `total_missions` | INTEGER | Completed mission count |

### `events`

| Column | Type | Description |
|---|---|---|
| `id` | INTEGER PK | Auto-increment |
| `drone_id` | INTEGER FK | Reference to drones |
| `event_type` | TEXT | HUMAN / VEHICLE / ACOUSTIC / DOCK / PATROL / COMMAND |
| `title` | TEXT | Short headline |
| `detail` | TEXT | Full description |
| `confidence` | REAL | AI confidence score (%) |
| `latitude` | REAL | Event location |
| `longitude` | REAL | Event location |
| `zone` | TEXT | Patrol zone label |
| `timestamp` | TEXT | UTC datetime |

### `detections`

| Column | Type | Description |
|---|---|---|
| `id` | INTEGER PK | Auto-increment |
| `drone_id` | INTEGER FK | Reference to drones |
| `object_type` | TEXT | HUMAN / VEHICLE / WEAPON |
| `confidence` | REAL | Detection confidence (%) |
| `latitude` | REAL | Detection location |
| `longitude` | REAL | Detection location |
| `zone` | TEXT | Patrol zone |
| `speed_ms` | REAL | Target speed (m/s) |
| `direction` | TEXT | Movement direction |
| `status` | TEXT | ACTIVE / STATIONARY / LOST |
| `timestamp` | TEXT | UTC datetime |

### `telemetry`

Auto-logged on every `/api/drones` call — builds a full time-series history.

| Column | Type | Description |
|---|---|---|
| `drone_id` | INTEGER FK | Reference to drones |
| `battery` | REAL | Battery SoC at snapshot |
| `altitude` | REAL | Altitude (m) |
| `latitude` | REAL | GPS latitude |
| `longitude` | REAL | GPS longitude |
| `speed` | REAL | Ground speed (m/s) |
| `heading` | REAL | Heading (degrees) |
| `wind_speed` | REAL | Simulated wind (m/s) |
| `wind_dir` | TEXT | Wind direction |
| `gps_sats` | INTEGER | Number of GPS satellites |
| `timestamp` | TEXT | UTC datetime |

---

## 🔌 REST API Reference

Base URL: `http://127.0.0.1:5000`

### Drones

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/api/drones` | Get all drones (triggers telemetry simulation tick) |
| `GET` | `/api/drones/<id>` | Get single drone by ID |
| `POST` | `/api/drones/<id>/command` | Send command to drone |

**Command payload:**
```json
{ "command": "RETURN" }
```

Available commands:

| Command | Effect |
|---|---|
| `RETURN` | Sets drone status to DOCKING |
| `LAUNCH` | Sets status to FLYING, altitude to 40m |
| `PAUSE` | Sets status to STANDBY, speed to 0 |
| `RESUME` | Sets status to FLYING |
| `EMERGENCY_ALL` | All FLYING drones → DOCKING |

### Events

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/api/events?limit=50` | Get event log (newest first) |
| `POST` | `/api/events` | Add new event |

**POST payload:**
```json
{
  "drone_id": 1,
  "event_type": "HUMAN",
  "title": "HUMAN DETECTED",
  "detail": "AI conf: 94% · Zone A2",
  "confidence": 94.0,
  "latitude": 39.9121,
  "longitude": 32.2834,
  "zone": "A2"
}
```

### Detections

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/api/detections` | Get latest detections |
| `POST` | `/api/detections` | Log new AI detection |

**POST payload:**
```json
{
  "drone_id": 1,
  "object_type": "HUMAN",
  "confidence": 94.0,
  "zone": "A2",
  "speed_ms": 2.3,
  "direction": "NE",
  "status": "ACTIVE"
}
```

### Telemetry

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/api/telemetry/<drone_id>?limit=60` | Get telemetry history for one drone |

### Session & Stats

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/api/session` | Session summary + active drone count + alert count |
| `PATCH` | `/api/session` | Update session stats |
| `GET` | `/api/stats` | Aggregate stats (event counts, avg battery, telemetry record count) |

---

##  Live Simulation

Every call to `GET /api/drones` triggers one simulation tick on the server:

```
FLYING  → battery drains ~0.02–0.06% per tick
          altitude drifts ±0.5 m
          position advances along heading
          speed varies ±0.3 m/s
          heading drifts ±3°
          → auto-transitions to DOCKING if battery < 20%

DOCKING → altitude decreases until grounded

CHARGING → battery charges ~0.3–0.6% per tick
           → auto-transitions to STANDBY at 95%
```

Each tick also writes one row to the `telemetry` table — building a full time-series history.

---

## 🧩 System Architecture (Hardware — Conceptual)

```
┌─────────────────────────────────────────────────────┐
│                   PHYSICAL LAYER                    │
│  450mm Carbon Fibre Frame  ·  4× BLDC + ESC         │
│  4S 5000 mAh LiPo  ·  ~30 min flight endurance      │
│  FLIR Lepton 3.5 Thermal Camera (160×120)           │
│  1080p RGB Wide-angle Camera                        │
│  4× MEMS Acoustic Microphone Array (TDOA)           │
│  Laser Rangefinder (docking precision)              │
└─────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────┐
│                    LOGIC LAYER                      │
│  GAP8 PULP-class MCU  ·  YOLOv5-nano  ·  64 mW     │
│  Pixhawk 4 Flight Controller                        │
│  PID Altitude Control  ·  GPS + IMU + Barometer     │
└─────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────┐
│             COMMUNICATION & INFRA                   │
│  AES-256 RF Module  ·  2.4 / 5.8 GHz               │
│  ArUco Vision Docking  ·  Qi Wireless Charging      │
│  GCS Web Interface (Flask + SQLite)                 │
└─────────────────────────────────────────────────────┘
```

---

## 📈 Simulation Results

| Metric | Result | Target | Status |
|---|---|---|---|
| Altitude settling time (±2%) | 4.3 s | ≤ 5 s | ✅ |
| Percentage overshoot | 3.2 % | < 5 % | ✅ |
| Max altitude deviation (wind ±1.5 N) | ±0.42 m | Acceptable | ✅ |
| Battery SoC at end of 30-min mission | 11.8 % | > 10 % | ✅ |
| Theoretical hover endurance | 31.25 min | ≥ 30 min | ✅ |
| System uptime | 100 % | ≥ 90 % | ✅ |

---

## 💰 Budget Estimate (Single Unit)

| Component | Est. Cost |
|---|---|
| Carbon-fibre frame (450mm) + motors + ESCs | ~9,800 ₺ |
| Pixhawk 4 flight controller + GPS | ~7,500 ₺ |
| FLIR Lepton 3.5 thermal camera | ~12,000 ₺ |
| 1080p RGB camera + MEMS mic array | ~4,900 ₺ |
| GAP8 PULP-class SoC board | ~6,300 ₺ |
| 2× 4S 5000 mAh LiPo batteries | ~4,200 ₺ |
| AES-256 RF module | ~5,250 ₺ |
| Docking station (ArUco + Qi) | ~10,500 ₺ |
| GCS workstation (one-time) | ~18,000 ₺ |
| Misc. hardware | ~2,800 ₺ |
| **TOTAL** | **~81,250 ₺ (~$2,200 USD)** |

---
 
**Project Supervisor:** Dr. Zühal Erden

---

##  References

- Shakhatreh et al. (2019). *UAVs: A Survey on Civil Applications and Key Research Challenges.* IEEE Access.
- Palossi et al. (2019). *A 64-mW DNN-based Visual Navigation Engine for Autonomous Nano-Drones.* IEEE IoT Journal.
- Lee et al. (2018). *Vision-based Autonomous Landing using Reinforcement Learning.* ICUAS 2018.
- Shi et al. (2020). *Acoustic-based Surveillance System for Drone Detection.* IEEE TVT.
- Arshad et al. (2022). *Drone Navigation using Deep CNN.* IEEE Access.

---

##  License

MIT License — see [LICENSE](LICENSE) for details.

---

<div align="center">

*Atılım University · MECE 322 Multidisciplinary Design in Engineering · May 2026*



</div>
