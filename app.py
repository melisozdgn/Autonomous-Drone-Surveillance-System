"""
ADSS – Autonomous Drone Surveillance System
Flask + SQLite Backend
"""

import sqlite3
import json
import random
import math
import time
from datetime import datetime, timezone
from flask import Flask, jsonify, request, render_template, g
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

DATABASE = "adss.db"

# ─────────────────────────────────────────────
#  DATABASE HELPERS
# ─────────────────────────────────────────────

def get_db():
    db = getattr(g, "_database", None)
    if db is None:
        db = g._database = sqlite3.connect(DATABASE)
        db.row_factory = sqlite3.Row
    return db

@app.teardown_appcontext
def close_connection(exception):
    db = getattr(g, "_database", None)
    if db is not None:
        db.close()

def query_db(query, args=(), one=False):
    cur = get_db().execute(query, args)
    rv = cur.fetchall()
    return (rv[0] if rv else None) if one else rv

def execute_db(query, args=()):
    db = get_db()
    db.execute(query, args)
    db.commit()


# ─────────────────────────────────────────────
#  DATABASE INIT
# ─────────────────────────────────────────────

def init_db():
    with app.app_context():
        db = get_db()
        db.executescript("""
            CREATE TABLE IF NOT EXISTS drones (
                id          INTEGER PRIMARY KEY,
                name        TEXT    NOT NULL,
                status      TEXT    DEFAULT 'STANDBY',
                battery     REAL    DEFAULT 100.0,
                altitude    REAL    DEFAULT 0.0,
                latitude    REAL    DEFAULT 39.9121,
                longitude   REAL    DEFAULT 32.2834,
                speed       REAL    DEFAULT 0.0,
                heading     REAL    DEFAULT 0.0,
                signal      TEXT    DEFAULT 'Strong',
                latency_ms  INTEGER DEFAULT 50,
                sector      TEXT    DEFAULT 'A1',
                flight_time INTEGER DEFAULT 0,
                total_missions INTEGER DEFAULT 0,
                created_at  TEXT    DEFAULT (datetime('now'))
            );

            CREATE TABLE IF NOT EXISTS events (
                id          INTEGER PRIMARY KEY AUTOINCREMENT,
                drone_id    INTEGER,
                event_type  TEXT    NOT NULL,
                title       TEXT    NOT NULL,
                detail      TEXT,
                confidence  REAL,
                latitude    REAL,
                longitude   REAL,
                zone        TEXT,
                timestamp   TEXT    DEFAULT (datetime('now')),
                FOREIGN KEY (drone_id) REFERENCES drones(id)
            );

            CREATE TABLE IF NOT EXISTS detections (
                id          INTEGER PRIMARY KEY AUTOINCREMENT,
                drone_id    INTEGER,
                object_type TEXT    NOT NULL,
                confidence  REAL    NOT NULL,
                latitude    REAL,
                longitude   REAL,
                zone        TEXT,
                speed_ms    REAL    DEFAULT 0.0,
                direction   TEXT,
                status      TEXT    DEFAULT 'ACTIVE',
                timestamp   TEXT    DEFAULT (datetime('now')),
                FOREIGN KEY (drone_id) REFERENCES drones(id)
            );

            CREATE TABLE IF NOT EXISTS telemetry (
                id          INTEGER PRIMARY KEY AUTOINCREMENT,
                drone_id    INTEGER,
                battery     REAL,
                altitude    REAL,
                latitude    REAL,
                longitude   REAL,
                speed       REAL,
                heading     REAL,
                wind_speed  REAL,
                wind_dir    TEXT,
                gps_sats    INTEGER,
                timestamp   TEXT    DEFAULT (datetime('now')),
                FOREIGN KEY (drone_id) REFERENCES drones(id)
            );

            CREATE TABLE IF NOT EXISTS dock_sessions (
                id            INTEGER PRIMARY KEY AUTOINCREMENT,
                drone_id      INTEGER,
                dock_name     TEXT,
                start_time    TEXT,
                end_time      TEXT,
                initial_soc   REAL,
                final_soc     REAL,
                precision_cm  REAL,
                status        TEXT    DEFAULT 'IN_PROGRESS',
                FOREIGN KEY (drone_id) REFERENCES drones(id)
            );

            CREATE TABLE IF NOT EXISTS session_stats (
                id              INTEGER PRIMARY KEY,
                start_time      TEXT    DEFAULT (datetime('now')),
                total_coverage  REAL    DEFAULT 0.0,
                dock_cycles     INTEGER DEFAULT 0,
                avg_response_s  REAL    DEFAULT 0.0,
                uptime_pct      REAL    DEFAULT 100.0,
                aes_uptime_pct  REAL    DEFAULT 100.0
            );
        """)
        db.commit()
        _seed_data(db)


def _seed_data(db):
    """Insert initial data if tables are empty."""
    # Drones
    if not db.execute("SELECT 1 FROM drones").fetchone():
        db.executemany(
            """INSERT INTO drones
               (id, name, status, battery, altitude, latitude, longitude,
                speed, heading, signal, latency_ms, sector, flight_time, total_missions)
               VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
            [
                (1, "DRONE 01", "FLYING",  78.0, 42.0, 39.9121, 32.2834, 4.2,  47.0, "Strong", 168, "A1-NE", 1122, 12),
                (2, "DRONE 02", "FLYING",  45.0, 38.0, 39.9098, 32.2867, 3.8, 220.0, "Fair",   120, "B1",    890,   9),
                (3, "DRONE 03", "DOCKING", 22.0,  5.0, 39.9135, 32.2851, 1.2,   0.0, "Fair",    74, "DOCK-A", 445,  7),
            ]
        )

    # Events
    if not db.execute("SELECT 1 FROM events").fetchone():
        db.executemany(
            """INSERT INTO events
               (drone_id, event_type, title, detail, confidence, latitude, longitude, zone, timestamp)
               VALUES (?,?,?,?,?,?,?,?,?)""",
            [
                (1, "HUMAN",   "HUMAN DETECTED",       "AI conf: 94% · Speed: 2.3 m/s NE · Tracking active",         94.0, 39.9121, 32.2834, "A2", "2026-05-31 08:42:31"),
                (1, "VEHICLE", "VEHICLE DETECTED",     "AI conf: 87% · Type: Light vehicle · Status: Stationary",    87.0, 39.9109, 32.2841, "A3", "2026-05-31 08:38:12"),
                (2, "ACOUSTIC","ACOUSTIC TRIGGER",     "Impact sound detected · TDOA localization · Drone activated", None, 39.9098, 32.2867, "B1", "2026-05-31 07:18:45"),
                (3, "DOCK",    "AUTO DOCK SUCCESS",    "Battery: 98% charged · Precision: ±3.2 cm · Charging OK",    None, 39.9135, 32.2851, "DOCK-A", "2026-05-31 06:56:02"),
                (1, "PATROL",  "PATROL CYCLE COMPLETE","Duration: 28.4 min · Coverage: 3.2 km² · No threats",        None, 39.9121, 32.2834, "A4",  "2026-05-31 06:11:30"),
            ]
        )

    # Detections
    if not db.execute("SELECT 1 FROM detections").fetchone():
        db.executemany(
            """INSERT INTO detections
               (drone_id, object_type, confidence, latitude, longitude, zone, speed_ms, direction, status, timestamp)
               VALUES (?,?,?,?,?,?,?,?,?,?)""",
            [
                (1, "HUMAN",   94.0, 39.9121, 32.2834, "A2", 2.3, "NE", "ACTIVE",   "2026-05-31 08:42:31"),
                (1, "VEHICLE", 87.0, 39.9109, 32.2841, "A3", 0.0, "—",  "STATIONARY","2026-05-31 08:38:12"),
            ]
        )

    # Session
    if not db.execute("SELECT 1 FROM session_stats").fetchone():
        db.execute(
            """INSERT INTO session_stats
               (id, start_time, total_coverage, dock_cycles, avg_response_s, uptime_pct, aes_uptime_pct)
               VALUES (1,'2026-05-31 02:00:00', 18.4, 4, 3.2, 97.0, 100.0)"""
        )

    db.commit()


# ─────────────────────────────────────────────
#  LIVE TELEMETRY SIMULATOR
# ─────────────────────────────────────────────

_sim_tick = 0

def simulate_telemetry():
    """Update drone telemetry with realistic values each call."""
    global _sim_tick
    _sim_tick += 1
    t = _sim_tick * 0.5

    db = get_db()
    drones = db.execute("SELECT * FROM drones").fetchall()

    for d in drones:
        did   = d["id"]
        st    = d["status"]
        bat   = d["battery"]
        alt   = d["altitude"]
        lat   = d["latitude"]
        lon   = d["longitude"]
        spd   = d["speed"]
        hdg   = d["heading"]
        lat_ms = d["latency_ms"]

        if st == "FLYING":
            bat   = max(0.0, bat - random.uniform(0.02, 0.06))
            alt   = max(30.0, min(80.0, alt + random.uniform(-0.5, 0.5)))
            lat   = lat + math.cos(math.radians(hdg)) * 0.000008
            lon   = lon + math.sin(math.radians(hdg)) * 0.000008
            spd   = max(2.0, min(8.0, spd + random.uniform(-0.3, 0.3)))
            hdg   = (hdg + random.uniform(-3, 3)) % 360
            lat_ms = max(50, min(300, lat_ms + random.randint(-5, 5)))
            ft    = d["flight_time"] + 1

            if bat < 20:
                st = "DOCKING"

            db.execute(
                """UPDATE drones SET battery=?,altitude=?,latitude=?,longitude=?,
                   speed=?,heading=?,latency_ms=?,status=?,flight_time=? WHERE id=?""",
                (round(bat,1), round(alt,1), round(lat,6), round(lon,6),
                 round(spd,1), round(hdg,1), lat_ms, st, ft, did)
            )

        elif st == "DOCKING":
            alt   = max(0.0, alt - random.uniform(0.5, 1.5))
            bat   = max(0.0, bat - 0.01)
            db.execute(
                "UPDATE drones SET altitude=?,battery=? WHERE id=?",
                (round(alt,1), round(bat,1), did)
            )

        elif st == "CHARGING":
            bat = min(100.0, bat + random.uniform(0.3, 0.6))
            if bat >= 95:
                st = "STANDBY"
            db.execute(
                "UPDATE drones SET battery=?,status=? WHERE id=?",
                (round(bat,1), st, did)
            )

        # Log telemetry snapshot
        wind = 1.0 + math.sin(t * 0.1) * 0.5
        db.execute(
            """INSERT INTO telemetry
               (drone_id,battery,altitude,latitude,longitude,speed,heading,wind_speed,wind_dir,gps_sats)
               VALUES (?,?,?,?,?,?,?,?,?,?)""",
            (did, round(bat,1), round(alt,1), round(lat,6), round(lon,6),
             round(spd,1), round(hdg,1), round(wind,2), "SW", random.randint(10,14))
        )

    db.commit()


# ─────────────────────────────────────────────
#  REST API ROUTES
# ─────────────────────────────────────────────

# ── Drones ──

@app.route("/api/drones", methods=["GET"])
def get_drones():
    simulate_telemetry()
    rows = query_db("SELECT * FROM drones ORDER BY id")
    return jsonify([dict(r) for r in rows])

@app.route("/api/drones/<int:did>", methods=["GET"])
def get_drone(did):
    row = query_db("SELECT * FROM drones WHERE id=?", (did,), one=True)
    if not row:
        return jsonify({"error": "Drone not found"}), 404
    return jsonify(dict(row))

@app.route("/api/drones/<int:did>/command", methods=["POST"])
def drone_command(did):
    data    = request.get_json() or {}
    command = data.get("command", "")
    db      = get_db()

    if command == "RETURN":
        db.execute("UPDATE drones SET status='DOCKING' WHERE id=?", (did,))
        msg = f"Drone {did} returning to base"
    elif command == "LAUNCH":
        db.execute("UPDATE drones SET status='FLYING',altitude=40.0 WHERE id=?", (did,))
        msg = f"Drone {did} launched"
    elif command == "PAUSE":
        db.execute("UPDATE drones SET status='STANDBY',speed=0 WHERE id=?", (did,))
        msg = f"Drone {did} paused"
    elif command == "RESUME":
        db.execute("UPDATE drones SET status='FLYING' WHERE id=?", (did,))
        msg = f"Drone {did} resumed patrol"
    elif command == "EMERGENCY_ALL":
        db.execute("UPDATE drones SET status='DOCKING' WHERE status='FLYING'")
        msg = "All drones returning to base (EMERGENCY)"
    else:
        return jsonify({"error": f"Unknown command: {command}"}), 400

    db.commit()
    _add_event(db, did, "COMMAND", f"CMD: {command}", msg)
    return jsonify({"ok": True, "message": msg})


# ── Events / Alert Log ──

@app.route("/api/events", methods=["GET"])
def get_events():
    limit = request.args.get("limit", 50, type=int)
    rows  = query_db(
        "SELECT e.*, d.name as drone_name FROM events e "
        "LEFT JOIN drones d ON e.drone_id=d.id "
        "ORDER BY e.id DESC LIMIT ?", (limit,)
    )
    return jsonify([dict(r) for r in rows])

@app.route("/api/events", methods=["POST"])
def add_event():
    data = request.get_json() or {}
    db   = get_db()
    _add_event(
        db,
        data.get("drone_id"),
        data.get("event_type", "INFO"),
        data.get("title", ""),
        data.get("detail", ""),
        data.get("confidence"),
        data.get("latitude"),
        data.get("longitude"),
        data.get("zone", ""),
    )
    return jsonify({"ok": True})

def _add_event(db, drone_id, etype, title, detail="",
               confidence=None, lat=None, lon=None, zone=""):
    db.execute(
        """INSERT INTO events (drone_id,event_type,title,detail,confidence,latitude,longitude,zone)
           VALUES (?,?,?,?,?,?,?,?)""",
        (drone_id, etype, title, detail, confidence, lat, lon, zone)
    )
    db.commit()


# ── Detections ──

@app.route("/api/detections", methods=["GET"])
def get_detections():
    rows = query_db(
        "SELECT d.*, dr.name as drone_name FROM detections d "
        "LEFT JOIN drones dr ON d.drone_id=dr.id "
        "ORDER BY d.id DESC LIMIT 20"
    )
    return jsonify([dict(r) for r in rows])

@app.route("/api/detections", methods=["POST"])
def add_detection():
    data = request.get_json() or {}
    execute_db(
        """INSERT INTO detections
           (drone_id,object_type,confidence,latitude,longitude,zone,speed_ms,direction,status)
           VALUES (?,?,?,?,?,?,?,?,?)""",
        (data.get("drone_id"), data.get("object_type"), data.get("confidence"),
         data.get("latitude"), data.get("longitude"), data.get("zone"),
         data.get("speed_ms", 0), data.get("direction", "—"), data.get("status","ACTIVE"))
    )
    return jsonify({"ok": True})


# ── Telemetry History ──

@app.route("/api/telemetry/<int:did>", methods=["GET"])
def get_telemetry(did):
    limit = request.args.get("limit", 60, type=int)
    rows  = query_db(
        "SELECT * FROM telemetry WHERE drone_id=? ORDER BY id DESC LIMIT ?",
        (did, limit)
    )
    return jsonify([dict(r) for r in reversed(rows)])


# ── Dock Sessions ──

@app.route("/api/dock_sessions", methods=["GET"])
def get_dock_sessions():
    rows = query_db(
        "SELECT ds.*, d.name as drone_name FROM dock_sessions ds "
        "LEFT JOIN drones d ON ds.drone_id=d.id ORDER BY ds.id DESC LIMIT 20"
    )
    return jsonify([dict(r) for r in rows])


# ── Session / Dashboard Summary ──

@app.route("/api/session", methods=["GET"])
def get_session():
    row    = query_db("SELECT * FROM session_stats WHERE id=1", one=True)
    flying = query_db("SELECT COUNT(*) as c FROM drones WHERE status='FLYING'", one=True)
    alerts = query_db("SELECT COUNT(*) as c FROM events WHERE event_type IN ('HUMAN','VEHICLE') "
                      "AND date(timestamp)=date('now')", one=True)
    det    = query_db("SELECT COUNT(*) as c FROM detections", one=True)
    result = dict(row) if row else {}
    result["active_drones"]   = flying["c"]
    result["alerts_today"]    = alerts["c"]
    result["total_detections"] = det["c"]
    return jsonify(result)

@app.route("/api/session", methods=["PATCH"])
def update_session():
    data = request.get_json() or {}
    db   = get_db()
    for key, val in data.items():
        if key in ("total_coverage","dock_cycles","avg_response_s","uptime_pct","aes_uptime_pct"):
            db.execute(f"UPDATE session_stats SET {key}=? WHERE id=1", (val,))
    db.commit()
    return jsonify({"ok": True})


# ── Stats endpoint ──

@app.route("/api/stats", methods=["GET"])
def get_stats():
    ev_counts = query_db(
        "SELECT event_type, COUNT(*) as cnt FROM events GROUP BY event_type"
    )
    bat_avg = query_db(
        "SELECT AVG(battery) as avg FROM drones WHERE status='FLYING'", one=True
    )
    telem_count = query_db("SELECT COUNT(*) as c FROM telemetry", one=True)
    return jsonify({
        "event_counts":    {r["event_type"]: r["cnt"] for r in ev_counts},
        "avg_battery_flying": round(bat_avg["avg"] or 0, 1),
        "telemetry_records":  telem_count["c"],
    })


# ── Frontend ──

@app.route("/")
def index():
    return render_template("index.html")


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────

if __name__ == "__main__":
    init_db()
    print("\n╔══════════════════════════════════════════╗")
    print("║   ADSS Ground Control Station — v1.0    ║")
    print("║   http://127.0.0.1:5000                  ║")
    print("╚══════════════════════════════════════════╝\n")
    app.run(debug=True, port=5000)
