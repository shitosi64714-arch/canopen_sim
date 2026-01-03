import canopen
import can

def send_sync():
    msg = can.Message(arbitration_id=0x80, data=[], is_extended_id=False)
    bus.send(msg)

print("SYNC test")
send_sync()
msg = bus.recv(timeout=0.1)
print("PDO:", msg)
