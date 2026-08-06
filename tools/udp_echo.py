"""
UDP echo test for PIOS.
Sends UDP packets to Pi at 192.168.222.222:9999 and listens for replies.

Usage:
  python udp_echo.py           # Listen for any UDP from Pi
  python udp_echo.py send MSG  # Send a UDP packet
  python udp_echo.py ping      # Send 5 test packets
"""
import socket, sys, time

PI_IP = "192.168.222.222"
PORT = 9999

def listen():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    sock.settimeout(30)
    print(f"Listening on UDP :{PORT}...")
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1500)
                print(f"[{addr[0]}:{addr[1]}] {len(data)}B: {data.decode('ascii','replace')}")
            except socket.timeout:
                print("(waiting...)")
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        sock.close()

def send(msg):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3)
    print(f"-> {PI_IP}:{PORT}: {msg!r}")
    sock.sendto(msg.encode(), (PI_IP, PORT))
    try:
        data, addr = sock.recvfrom(1500)
        print(f"<- {addr[0]}:{addr[1]}: {data.decode('ascii','replace')}")
    except socket.timeout:
        print("No reply")
    sock.close()

def ping():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2)
    for i in range(5):
        msg = f"PIOS_TEST_{i}"
        sock.sendto(msg.encode(), (PI_IP, PORT))
        try:
            data, addr = sock.recvfrom(1500)
            print(f"[{i}] Reply: {data.decode('ascii','replace')}")
        except socket.timeout:
            print(f"[{i}] Timeout")
        time.sleep(0.5)
    sock.close()

if __name__ == "__main__":
    if len(sys.argv) < 2: listen()
    elif sys.argv[1] == "send": send(" ".join(sys.argv[2:]))
    elif sys.argv[1] == "ping": ping()
    else: print(__doc__)
