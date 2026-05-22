from socketcan import CanRawSocket, CanFrame
import time

interface = "vcan0"
s = CanRawSocket(interface=interface)

# can data configuration:
#     01: Speed         - 1 byte
#     02: gear          - 1 byte
#     03: handBreak     - 1 byte
#     04: fuel          - 1 byte
#     05: throttle      - 1 byte
#     06: break         - 1 byte
#     07: elappsed_time - 3 byte

def send_canframe(can_id, data_list):
    data = bytes(data_list)
    frame1 = CanFrame(can_id=can_id, data=data)
    s.send(frame1)

speed = 0
gear = 0
handBreak = 0
fuel=0
throttleVal = 0
breakVal = 0
elappsed_time = 0

while True:
    elappsed_time += 1

    send_canframe(0x01, [speed])
    send_canframe(0x02, [gear])
    send_canframe(0x03, [handBreak])
    send_canframe(0x04, [fuel])
    send_canframe(0x05, [throttleVal])
    send_canframe(0x06, [breakVal])
    send_canframe(0x07, [elappsed_time//3600%250, elappsed_time//60%60, elappsed_time%60])

    # Debug
    speed += 2
    if speed >= 200:
        speed = 0
    gear = (1+ speed // 20) % 5
    handBreak = 1 if (speed // 25) % 2 == 0 else 0
    fuel = (speed * 100 // 200) % 100
    throttleVal = speed * 100 // 200
    breakVal = speed * 100 // 200

    time.sleep(0.05)

