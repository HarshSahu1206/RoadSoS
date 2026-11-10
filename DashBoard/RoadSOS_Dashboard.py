import csv
import json
import re
import socket
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

from flask import Flask, jsonify, render_template_string, request

# ============================================================
# CONFIGURATION
# ============================================================
TCP_HOST = "0.0.0.0"
TCP_PORT = 5000
WEB_HOST = "0.0.0.0"
WEB_PORT = 5001

LABELS = {
    "unlabeled", "normal", "bump", "hard_braking",
    "turn", "accident", "other"
}

# Exact column names confirmed from the user's ESP32 CSV header.
EXPECTED_FIELDS = [
    "timestamp_ms", "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps",
    "roll_deg", "pitch_deg", "tilt_deg", "accel_mag_g",
    "gps_fix", "satellites", "latitude", "longitude",
    "speed_kmph", "course_deg", "impact_flag", "event_label"
]

stop_event = threading.Event()
conn_lock = threading.Lock()
data_lock = threading.Lock()
current_conn = None

latest_record = {}
recent_records = deque(maxlen=300)
event_history = deque(maxlen=50)
status_messages = deque(maxlen=20)
stats = {
    "connected": False,
    "rows_saved": 0,
    "valid_rows_per_second": 0.0,
    "dropped_samples": None,
    "last_seen": None,
    "csv_file": "",
    "csv_ready": False,
    "header_fields": 0,
}
row_times = deque(maxlen=100)
last_impact_state = False
last_event_label = "unlabeled"
record_version = 0
server_started_at = time.time()

app = Flask(__name__)

# ============================================================
# DASHBOARD PAGE
# ============================================================
PAGE = r"""
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RoadSOS | Vehicle Safety Analytics</title>
<link rel="preconnect" href="https://unpkg.com">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
:root{--ink:#172b4d;--muted:#6b7a90;--blue:#2563eb;--pale:#eff6ff;--line:#e5eaf1;--bg:#f5f8fc;--green:#15966b;--red:#dc4444;--amber:#d28a16}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px Inter,Segoe UI,Arial,sans-serif}
header{background:#fff;border-bottom:1px solid var(--line);padding:17px 26px;display:flex;justify-content:space-between;align-items:center;gap:15px;position:sticky;top:0;z-index:500}
.brand{font-size:20px;font-weight:750;letter-spacing:-.4px}.brand span{color:var(--blue)}.sub{color:var(--muted);font-size:12px;margin-top:4px}
.pill{border:1px solid var(--line);border-radius:30px;padding:8px 12px;background:#fff;color:var(--muted);font-size:12px}.dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#aab4c2;margin-right:7px}.dot.on{background:#16a36f}
main{max-width:1440px;margin:auto;padding:24px}.section-title{font-size:16px;font-weight:700;margin:4px 0 13px}.metrics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:14px}
.card{background:#fff;border:1px solid var(--line);border-radius:13px;padding:17px;box-shadow:0 2px 8px #1d35570a;min-width:0}
.metric-label{font-size:12px;color:var(--muted)}.metric-value{font-size:27px;font-weight:750;margin:10px 0 5px;letter-spacing:-.6px}.metric-foot{font-size:11px;color:var(--muted)}
.layout{display:grid;grid-template-columns:minmax(0,1.55fr) minmax(300px,.85fr);gap:15px;margin-top:17px}
#map{height:330px;border-radius:9px;background:#edf2f7;z-index:1}.map-meta{font-size:12px;color:var(--muted);margin-top:10px}
.chart-grid{display:grid;grid-template-columns:1fr 1fr;gap:15px;margin-top:15px}.chart-wrap{height:220px;position:relative}
.event-list{display:flex;flex-direction:column;gap:9px;max-height:265px;overflow:auto}.event{border:1px solid var(--line);border-radius:9px;padding:10px 11px;display:flex;justify-content:space-between;gap:12px}.event strong{display:block;font-size:12px}.event small{color:var(--muted)}
.tag{font-size:10px;border-radius:20px;padding:4px 8px;background:#fff7e8;color:#a56a0b;height:max-content}.tag.red{background:#fff0f0;color:#c23535}.tag.blue{background:#eff6ff;color:#245bc4}
.diag-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}.diag{padding:12px;background:#f8fafc;border-radius:9px}.diag b{display:block;font-size:16px;margin-top:7px}.diag span{color:var(--muted);font-size:11px}
.label-row{display:flex;gap:9px;align-items:center;flex-wrap:wrap;margin-top:12px}select,button{border:1px solid var(--line);border-radius:8px;padding:9px 11px;background:white;color:var(--ink)}button{background:var(--blue);border-color:var(--blue);color:white;font-weight:650;cursor:pointer}button:hover{background:#1d4ed8}
.footer{font-size:11px;color:var(--muted);text-align:center;padding:24px}
@media(max-width:900px){.metrics{grid-template-columns:repeat(2,1fr)}.layout{grid-template-columns:1fr}.diag-grid{grid-template-columns:repeat(2,1fr)}}
@media(max-width:560px){header{padding:13px 15px}main{padding:15px}.chart-grid{grid-template-columns:1fr}.metrics{gap:9px}.metric-value{font-size:23px}}
</style>
</head>
<body>
<header>
 <div><div class="brand">Road<span>SOS</span> <span style="color:#6b7a90;font-weight:500">/ Vehicle Analytics</span></div>
 <div class="sub">Real-time vehicle safety monitoring</div></div>
 <div class="pill"><i class="dot" id="connectionDot"></i><span id="connectionText">Waiting for ESP32</span></div>
</header>
<main>
 <div class="section-title">Live overview</div>
 <section class="metrics">
  <div class="card"><div class="metric-label">Vehicle speed</div><div class="metric-value" id="speed">-- km/h</div><div class="metric-foot">GPS speed</div></div>
  <div class="card"><div class="metric-label">Vehicle condition</div><div class="metric-value" id="condition" style="font-size:23px">Waiting</div><div class="metric-foot" id="conditionFoot">No current event</div></div>
  <div class="card"><div class="metric-label">Acceleration magnitude</div><div class="metric-value" id="accel">-- g</div><div class="metric-foot">IMU measurement</div></div>
  <div class="card"><div class="metric-label">GPS signal</div><div class="metric-value" id="gps">No fix</div><div class="metric-foot" id="satellites">-- satellites</div></div>
 </section>

 <section class="layout">
  <div class="card"><div class="section-title">Live vehicle location</div><div id="map"></div><div class="map-meta" id="coordinates">Waiting for a valid GPS fix...</div></div>
  <div class="card"><div class="section-title">Event history</div><div class="event-list" id="events"><div class="map-meta">No events received yet.</div></div>
   <div class="label-row"><label for="labelSelect" class="metric-label">Send event label to ESP32</label>
    <select id="labelSelect"><option>normal</option><option>bump</option><option>hard_braking</option><option>turn</option><option>accident</option><option>other</option><option>unlabeled</option></select>
    <button onclick="sendLabel()">Send label</button></div><div class="map-meta" id="labelMessage">Manual labels are for event annotation, not automatic accident confirmation.</div>
  </div>
 </section>

 <section class="chart-grid">
  <div class="card"><div class="section-title">Acceleration (g)</div><div class="chart-wrap"><canvas id="accelChart"></canvas></div></div>
  <div class="card"><div class="section-title">Gyroscope (°/s)</div><div class="chart-wrap"><canvas id="gyroChart"></canvas></div></div>
  <div class="card"><div class="section-title">Vehicle orientation</div><div class="chart-wrap"><canvas id="tiltChart"></canvas></div></div>
  <div class="card"><div class="section-title">Speed (km/h)</div><div class="chart-wrap"><canvas id="speedChart"></canvas></div></div>
 </section>

 <section class="card" style="margin-top:15px"><div class="section-title">Technical diagnostics</div>
  <div class="diag-grid">
   <div class="diag"><span>ESP32 connection</span><b id="diagConnection">Waiting</b></div>
   <div class="diag"><span>Rows recorded</span><b id="rows">0</b></div>
   <div class="diag"><span>Receive rate</span><b id="rate">0.0 rows/s</b></div>
   <div class="diag"><span>Dropped samples</span><b id="dropped">Not reported</b></div>
   <div class="diag"><span>CSV recording</span><b id="csvStatus">Starting</b></div>
   <div class="diag"><span>GPS fix</span><b id="gpsDiag">Unavailable</b></div>
   <div class="diag"><span>Last data received</span><b id="lastSeen">--</b></div>
   <div class="diag"><span>CSV file</span><b id="csvName" style="font-size:11px;overflow-wrap:anywhere">--</b></div>
  </div>
 </section>
 <div class="footer">RoadSOS prototype • Sensor alerts are not a certified accident diagnosis or safety guarantee.</div>
</main>
<script>
const $ = id => document.getElementById(id);
const map = L.map('map').setView([20.5937,78.9629],5);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'}).addTo(map);
let marker=null, route=L.polyline([],{color:'#2563eb',weight:4}).addTo(map), centered=false;
function makeChart(id, datasets) {
 return new Chart($(id),{type:'line',data:{labels:[],datasets:datasets.map(d=>({...d,data:[],borderWidth:2,pointRadius:0,tension:.25,fill:false}))},
 options:{responsive:true,maintainAspectRatio:false,animation:false,interaction:{intersect:false,mode:'index'},
 plugins:{legend:{labels:{boxWidth:10,font:{size:10}}}},scales:{x:{display:false},y:{grid:{color:'#edf1f6'},ticks:{font:{size:10}}}}}});
}
const charts={
 accel:makeChart('accelChart',[{label:'X',borderColor:'#2563eb'},{label:'Y',borderColor:'#14b8a6'},{label:'Z',borderColor:'#f59e0b'}]),
 gyro:makeChart('gyroChart',[{label:'GX',borderColor:'#2563eb'},{label:'GY',borderColor:'#14b8a6'},{label:'GZ',borderColor:'#f59e0b'}]),
 tilt:makeChart('tiltChart',[{label:'Roll',borderColor:'#2563eb'},{label:'Pitch',borderColor:'#14b8a6'},{label:'Tilt',borderColor:'#f59e0b'}]),
 speed:makeChart('speedChart',[{label:'Speed',borderColor:'#2563eb'}])
};
let lastVersion = -1;
function num(v){const n=Number(v);return v===null||v===undefined||v===""?null:(Number.isFinite(n)?n:null)}
function updateChart(chart, label, values){
 chart.data.labels.push(label); chart.data.datasets.forEach((ds,i)=>ds.data.push(values[i]));
 while(chart.data.labels.length>80){chart.data.labels.shift();chart.data.datasets.forEach(ds=>ds.data.shift())}
 chart.update('none');
}
function renderEvents(events){
 $('events').innerHTML='';
 if(!events.length){$('events').innerHTML='<div class="map-meta">No events received yet.</div>';return}
 events.slice().reverse().forEach(e=>{
  const div=document.createElement('div');div.className='event';
  const left=document.createElement('div');const strong=document.createElement('strong');strong.textContent=e.label;
  const small=document.createElement('small');small.textContent=e.time+' • '+(e.location||'Location unavailable');
  left.append(strong,small);const tag=document.createElement('span');tag.className='tag '+(e.impact?'red':'blue');tag.textContent=e.impact?'Impact flag':'Label';
  div.append(left,tag);$('events').append(div);
 });
}
async function refresh(){
 try{
  const r=await fetch('/api/data');const d=await r.json();const x=d.latest||{};
  const connected=d.stats.connected;
  $('connectionDot').className='dot '+(connected?'on':'');$('connectionText').textContent=connected?'ESP32 connected':'Waiting for ESP32';
  $('diagConnection').textContent=connected?'Connected':'Disconnected';
  $('rows').textContent=d.stats.rows_saved;$('rate').textContent=d.stats.valid_rows_per_second.toFixed(1)+' rows/s';
  $('dropped').textContent=d.stats.dropped_samples===null?'Not reported':d.stats.dropped_samples;
  $('csvStatus').textContent=d.stats.csv_ready?'Recording':'Waiting for header';
  $('csvName').textContent=d.stats.csv_file||'--';
  $('lastSeen').textContent=d.stats.last_seen||'--';
  $('speed').textContent=(num(x.speed_kmph)??'--')+' km/h';
  $('accel').textContent=(num(x.accel_mag_g)??'--')+' g';
  const fix=String(x.gps_fix)==='1'&&num(x.latitude)!==null&&num(x.longitude)!==null;
  $('gps').textContent=fix?'GPS fixed':'No fix';$('satellites').textContent=(x.satellites??'--')+' satellites';
  $('gpsDiag').textContent=fix?'Valid fix':'Unavailable';
  const label=x.event_label||'unlabeled', impact=String(x.impact_flag)==='1';
  $('condition').textContent=impact?'Check event':(label==='unlabeled'?'Monitoring':label.replaceAll('_',' '));
  $('condition').style.color=impact?'#dc4444':'#15966b';
  $('conditionFoot').textContent=impact?'High-acceleration flag received':'Latest ESP32 event label';
  const isNewRecord = d.version !== lastVersion;
  if(isNewRecord && fix){
   const lat=Number(x.latitude),lon=Number(x.longitude);
   $('coordinates').textContent=`Latitude ${lat.toFixed(6)} • Longitude ${lon.toFixed(6)}`;
   const p=[lat,lon];route.addLatLng(p);
   if(!marker){marker=L.marker(p).addTo(map).bindPopup('Current vehicle position')}
   else marker.setLatLng(p);
   if(!centered){map.setView(p,16);centered=true}
  }else{$('coordinates').textContent='Waiting for a valid GPS fix...'}
  if(isNewRecord){
   lastVersion = d.version;
   const t=new Date().toLocaleTimeString();
   updateChart(charts.accel,t,[num(x.ax_g),num(x.ay_g),num(x.az_g)]);
   updateChart(charts.gyro,t,[num(x.gx_dps),num(x.gy_dps),num(x.gz_dps)]);
   updateChart(charts.tilt,t,[num(x.roll_deg),num(x.pitch_deg),num(x.tilt_deg)]);
   updateChart(charts.speed,t,[num(x.speed_kmph)]);
  }
  renderEvents(d.events||[]);
 }catch(e){$('connectionText').textContent='Dashboard API unavailable'}
}
async function sendLabel(){
 const label=$('labelSelect').value;
 try{const r=await fetch('/api/label',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({label})});
 const d=await r.json();$('labelMessage').textContent=d.message||d.error||'Request sent';}
 catch(e){$('labelMessage').textContent='Could not contact dashboard server'}
}
setInterval(refresh,1000);refresh();
</script>
</body>
</html>
"""

# ============================================================
# HELPERS
# ============================================================
def get_local_ips():
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


def safe_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def add_record(record):
    global last_impact_state, last_event_label, record_version
    now = time.time()
    label = (record.get("event_label") or "unlabeled").strip()
    impact = str(record.get("impact_flag", "0")).strip() == "1"
    lat = record.get("latitude", "").strip()
    lon = record.get("longitude", "").strip()
    valid_gps = record.get("gps_fix") == "1" and lat and lon
    location = f"{lat}, {lon}" if valid_gps else ""

    with data_lock:
        latest_record.clear()
        latest_record.update(record)
        recent_records.append(record)
        record_version += 1
        row_times.append(now)
        stats["rows_saved"] += 1
        stats["last_seen"] = datetime.now().strftime("%H:%M:%S")
        if len(row_times) >= 2:
            elapsed = row_times[-1] - row_times[0]
            stats["valid_rows_per_second"] = round((len(row_times) - 1) / elapsed, 1) if elapsed > 0 else 0.0

        # Record an event only on a new impact flag or a label transition.
        if (impact and not last_impact_state) or (label != last_event_label and label != "unlabeled"):
            event_history.appendleft({
                "label": "Possible high-acceleration event" if impact else label.replace("_", " "),
                "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "location": location,
                "impact": impact,
            })
        last_impact_state = impact
        last_event_label = label


def parse_status_line(line):
    with data_lock:
        status_messages.append({"time": datetime.now().strftime("%H:%M:%S"), "message": line})
        match = re.search(r"dropped\s*=\s*(\d+)", line, re.IGNORECASE)
        if match:
            stats["dropped_samples"] = int(match.group(1))


# ============================================================
# FLASK ROUTES
# ============================================================
@app.get("/")
def dashboard():
    return render_template_string(PAGE)


@app.get("/api/data")
def api_data():
    with data_lock:
        payload = {
            "latest": dict(latest_record),
            "events": list(event_history),
            "stats": dict(stats),
            "version": record_version,
        }
    return jsonify(payload)


@app.post("/api/label")
def api_label():
    data = request.get_json(silent=True) or {}
    label = str(data.get("label", "")).strip().lower()
    if label not in LABELS:
        return jsonify({"error": "Invalid label"}), 400
    with conn_lock:
        conn = current_conn
        if conn is None:
            return jsonify({"error": "ESP32 is not connected"}), 503
        try:
            conn.sendall(f"LABEL={label}\n".encode("ascii"))
        except OSError as exc:
            return jsonify({"error": f"Could not send label: {exc}"}), 500
    return jsonify({"message": f"Label command sent: {label}"})


# ============================================================
# TCP RECEIVER + CSV LOGGER
# ============================================================
def tcp_receiver():
    global current_conn
    filename = "car_data_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    output_path = Path.cwd() / filename
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind((TCP_HOST, TCP_PORT))
    except OSError as exc:
        print(f"[TCP] Could not bind to port {TCP_PORT}: {exc}")
        print("[TCP] Another program may already be using this port.")
        return

    server.listen(1)
    server.settimeout(1.0)
    print(f"[TCP] Listening on {TCP_HOST}:{TCP_PORT}")
    ips = get_local_ips()
    if ips:
        print("[TCP] Laptop IPv4 address(es):", ", ".join(ips))
        print("[TCP] Ensure SERVER_IP in ESP32 code matches this laptop's Wi-Fi IP.")
    print("[TCP] Waiting for ESP32...")

    header = None
    writer = None
    rows_saved_local = 0

    try:
        with open(output_path, "w", newline="", encoding="utf-8") as csv_file:
            with data_lock:
                stats["csv_file"] = output_path.name
                stats["csv_ready"] = True

            while not stop_event.is_set():
                try:
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                conn.settimeout(1.0)
                with conn_lock:
                    current_conn = conn
                with data_lock:
                    stats["connected"] = True
                print(f"[TCP] ESP32 connected from {addr[0]}:{addr[1]}")
                buffer = b""

                try:
                    while not stop_event.is_set():
                        try:
                            chunk = conn.recv(4096)
                        except socket.timeout:
                            continue
                        if not chunk:
                            print("[TCP] ESP32 disconnected; waiting for reconnect...")
                            break
                        buffer += chunk

                        while b"\n" in buffer:
                            raw, buffer = buffer.split(b"\n", 1)
                            line = raw.decode("utf-8", errors="replace").strip()
                            if line.startswith("#"):
                                print(line)
                                parse_status_line(line)
                                continue
                            if not line:
                                continue

                            if line.startswith("timestamp_ms,"):
                                new_header = next(csv.reader([line]))
                                if header is None:
                                    header = new_header
                                    writer = csv.writer(csv_file)
                                    writer.writerow(header)
                                    csv_file.flush()
                                    with data_lock:
                                        stats["header_fields"] = len(header)
                                    print(f"[CSV] Recording to: {output_path}")
                                    print(f"[CSV] Columns detected: {len(header)}")
                                    if header != EXPECTED_FIELDS:
                                        print("[CSV] NOTE: Header differs from the expected 19-column header.")
                                continue

                            if header is None or writer is None:
                                continue
                            try:
                                row = next(csv.reader([line]))
                            except csv.Error:
                                continue
                            if len(row) != len(header):
                                continue

                            record = dict(zip(header, row))
                            writer.writerow(row)
                            rows_saved_local += 1
                            if rows_saved_local % 50 == 0:
                                csv_file.flush()
                            if rows_saved_local % 500 == 0:
                                print(f"[CSV] Rows saved: {rows_saved_local}")
                            add_record(record)

                            if record.get("impact_flag") == "1":
                                print("[EVENT] Possible high-acceleration event:",
                                      "accel_mag_g =", record.get("accel_mag_g"),
                                      "g | label =", record.get("event_label"))
                except (ConnectionResetError, ConnectionAbortedError, OSError) as exc:
                    print(f"[TCP] Connection error: {exc}")
                finally:
                    with conn_lock:
                        current_conn = None
                    with data_lock:
                        stats["connected"] = False
                    try:
                        conn.close()
                    except OSError:
                        pass
            csv_file.flush()
    finally:
        server.close()
        print(f"[TCP] Receiver stopped. Rows saved: {rows_saved_local}")
        print(f"[CSV] File: {output_path}")


def main():
    receiver_thread = threading.Thread(target=tcp_receiver, daemon=True)
    receiver_thread.start()
    ips = get_local_ips()
    print("\n[WEB] Starting RoadSOS dashboard...")
    print(f"[WEB] Open on this laptop: http://127.0.0.1:{WEB_PORT}")
    for ip in ips:
        print(f"[WEB] Network address: http://{ip}:{WEB_PORT}")
    print("[WEB] Keep this terminal open while using the dashboard.")
    try:
        app.run(host=WEB_HOST, port=WEB_PORT, debug=False, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        with conn_lock:
            conn = current_conn
        if conn:
            try:
                conn.shutdown(socket.SHUT_RDWR)
                conn.close()
            except OSError:
                pass
        print("\nRoadSOS stopped.")


if __name__ == "__main__":
    main()
