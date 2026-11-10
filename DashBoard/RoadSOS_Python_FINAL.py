import csv
import socket
import threading
from datetime import datetime
from pathlib import Path

# ==================================================
# CONFIGURATION
# ==================================================

HOST = "0.0.0.0"   # listen on all network interfaces (required for ESP32)
PORT = 5000        # must match SERVER_PORT in the ESP32 code


LABELS = {
    "unlabeled",
    "normal",
    "bump",
    "hard_braking",
    "turn",
    "accident",
    "other",
}

stop_event = threading.Event()
conn_lock = threading.Lock()
current_conn = None  # active ESP32 connection (used to send labels)


# ==================================================
# HELPERS
# ==================================================

def get_local_ips():
    """Print this laptop's IPv4 addresses so SERVER_IP can be checked."""
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


# ==================================================
# LABEL CONTROL (sends commands to ESP32 over Wi-Fi)
# ==================================================

def label_input():
    print("\nAvailable labels:")
    print("normal, bump, hard_braking, turn, accident, other, unlabeled")
    print("Type 'quit' to stop recording.\n")

    while not stop_event.is_set():
        try:
            command = input("Label> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            stop_event.set()
            break

        if command == "quit":
            stop_event.set()
            break

        if command not in LABELS:
            print("Invalid label. Choose:", ", ".join(sorted(LABELS)))
            continue

        with conn_lock:
            conn = current_conn

        if conn is None:
            print("ESP32 is not connected right now.")
            continue

        try:
            conn.sendall(f"LABEL={command}\n".encode("ascii"))
            print(f"Label requested: {command}")
        except OSError as exc:
            print(f"Could not send label: {exc}")


# ==================================================
# RECORD HANDLING
# ==================================================

def report_impact(record):
    print("\n*** POSSIBLE HIGH-ACCELERATION EVENT ***")
    print("ESP32 timestamp:", record.get("timestamp_ms"))
    print("Current label:", record.get("event_label"))
    print("Acceleration:", record.get("accel_mag_g"), "g")

    if (
        record.get("gps_fix") == "1"
        and record.get("latitude", "").strip()
        and record.get("longitude", "").strip()
    ):
        print("Latitude:", record["latitude"])
        print("Longitude:", record["longitude"])
    else:
        print("GPS location unavailable: no fresh valid GPS fix.")

    print("***************************************\n")


# ==================================================
# MAIN LOGGER
# ==================================================

def main():
    global current_conn

    filename = "car_data_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    output_path = Path.cwd() / filename

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind((HOST, PORT))
    except OSError as exc:
        print(f"Could not bind to port {PORT}: {exc}")
        print("Another program may be using this port.")
        return

    server.listen(1)
    server.settimeout(1.0)

    print(f"Listening on {HOST}:{PORT}")
    ips = get_local_ips()
    if ips:
        print("Laptop IPv4 address(es):", ", ".join(ips))
        print("Make sure SERVER_IP in the ESP32 code matches one of these.")
    print("Waiting for ESP32 to connect...")

    header = None
    writer = None
    rows_saved = 0

    try:
        with open(output_path, "w", newline="", encoding="utf-8") as csv_file:
            threading.Thread(target=label_input, daemon=True).start()

            while not stop_event.is_set():
                # ---- wait for the ESP32 to (re)connect ----
                try:
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                conn.settimeout(1.0)
                with conn_lock:
                    current_conn = conn
                print(f"\nESP32 connected from {addr[0]}:{addr[1]}")

                buffer = b""

                # ---- receive data from this connection ----
                try:
                    while not stop_event.is_set():
                        try:
                            chunk = conn.recv(4096)
                        except socket.timeout:
                            continue

                        if not chunk:
                            print("ESP32 disconnected. Waiting for reconnect...")
                            break

                        buffer += chunk

                        while b"\n" in buffer:
                            raw, buffer = buffer.split(b"\n", 1)
                            line = raw.decode("utf-8", errors="replace").strip()

                            # Show ESP32 status lines (e.g. the control page address).
                            if line.startswith("#"):
                                print(line)
                                continue
                            if not line:
                                continue

                            # CSV header from the ESP32.
                            if line.startswith("timestamp_ms,"):
                                new_header = next(csv.reader([line]))
                                if header is None:
                                    header = new_header
                                    writer = csv.writer(csv_file)
                                    writer.writerow(header)
                                    csv_file.flush()
                                    print(f"Recording to:\n{output_path}")
                                    print(f"Columns detected: {len(header)}")
                                    print("Data collection is running.")
                                # On reconnect the header is repeated; skip it
                                # so the CSV keeps a single header row.
                                continue

                            if header is None or writer is None:
                                continue

                            try:
                                row = next(csv.reader([line]))
                            except csv.Error:
                                continue

                            # Reject incomplete or malformed records.
                            if len(row) != len(header):
                                continue

                            writer.writerow(row)
                            rows_saved += 1

                            if rows_saved % 50 == 0:
                                csv_file.flush()
                            if rows_saved % 500 == 0:
                                print(f"Rows saved: {rows_saved}")

                            record = dict(zip(header, row))
                            if record.get("impact_flag") == "1":
                                report_impact(record)

                except (ConnectionResetError, ConnectionAbortedError, OSError) as exc:
                    print(f"Connection error: {exc}. Waiting for reconnect...")

                finally:
                    with conn_lock:
                        current_conn = None
                    try:
                        conn.close()
                    except OSError:
                        pass

            csv_file.flush()

    except KeyboardInterrupt:
        stop_event.set()

    finally:
        stop_event.set()
        server.close()
        print("\nRecording stopped.")
        print(f"Rows saved: {rows_saved}")
        print(f"CSV file: {output_path}")


if __name__ == "__main__":
    main()
