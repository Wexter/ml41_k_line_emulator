import serial

import time

SERIAL_DEVICE = '/dev/ttyUSB0'

# linux ch341 module works very strange, on 8860 baud it sets real serial speed at half of requested, but works fine with ~8830-8850 baud rate
serial_port = serial.Serial(SERIAL_DEVICE, 8840, timeout=2)

def esp_reset():
    # Reset
    serial_port.setDTR(False)

    time.sleep(1)

    serial_port.setDTR(True)

def init_connection():
    serial_port.send_break(1)
    time.sleep(0.2)
    serial_port.send_break(0.6)
    time.sleep(0.1)

def read_byte(send_echo: bool = False):
    input_byte = serial_port.read(1)

    # print(f'RX: {input_byte.hex()}')

    if send_echo:
        reply = (0xFF - int.from_bytes(input_byte, 'big')).to_bytes(1, 'big')

        # print(f'TXe: {reply.hex()}')

        serial_port.write(reply)

        serial_port.read(1)

    return input_byte

def send_byte(send_byte, recv_echo: bool = False):
    # print(f'TX: {send_byte}')

    serial_port.write(send_byte.to_bytes(1, 'big'))

    time.sleep(0.01)

    serial_port.read(1)

    # print(f'DUP: {serial_port.read(1).hex(" ")}')

    if recv_echo:
        echo_byte = serial_port.read(1)
        # print(f'RXe: {echo_byte.hex()}')

def recv_packet():
    packet = bytearray(read_byte(True))

    packet_length = packet[0]

    idx = 1

    while idx <= packet_length:
        # print(f'{idx}/{packet_length}')
        new_byte = read_byte(idx != packet_length)
        packet.append(int.from_bytes(new_byte, 'big'))
        idx = idx + 1

    print(packet.hex(' '))

    return packet

def send_packet(packet: bytearray):
    print(packet.hex(' '))

    idx = 0

    while idx <= packet[0]:
        send_byte(packet[idx], idx != packet[0])
        idx = idx + 1

# print('---- reset esp32 ----')
# esp_reset()

# time.sleep(2)

# serial_port.reset_input_buffer()

# serial_port.reset_output_buffer()

print('---- init connection ----')

init_connection()

time.sleep(0.4)

print('---- sync & kw ----')

read_byte(False)

read_byte(True)

read_byte(True)

time.sleep(0.02)

print('---- recv_packet ----')

recv_packet()

time.sleep(0.02)

print('---- send_packet ----')

send_packet(bytearray(b'\x03\x02\x09\x03'))

print('---- recv_packet ----')

recv_packet()

time.sleep(0.02)

print('---- send_packet ----')

send_packet(bytearray(b'\x03\x04\x09\x03'))

print('---- recv_packet ----')

recv_packet()

time.sleep(0.02)

print('---- send_packet ----')

send_packet(bytearray(b'\x03\x06\x09\x03'))

print('---- recv_packet ----')

recv_packet()

test_requests = [
#    bytearray(b'\x06\x00\x03\x0D\x00\x00\x03'), # read EPROM
   bytearray(b'\x04\x00\x08\x00\x03'), # get AFR
   bytearray(b'\x04\x00\x08\x01\x03'), # get Vbat
   bytearray(b'\x04\x00\x08\x02\x03'), # get intake air temp
   bytearray(b'\x04\x00\x08\x03\x03'), # get coolant temp
   bytearray(b'\x03\x00\x05\x03'), # erase error codes
   bytearray(b'\x04\x00\x08\x04\x03'), # get CO pot
   bytearray(b'\x04\x00\x08\x05\x03'), # get O2 sensor
   bytearray(b'\x04\x00\x08\x07\x03'), # get ignition time
   bytearray(b'\x03\x00\x07\x03'), # get error codes
   bytearray(b'\x06\x00\x01\x01\x00\x3a\x03'), # get RPM
   bytearray(b'\x06\x00\x01\x01\x00\x20\x03'), # get TPS
   bytearray(b'\x06\x00\x01\x01\x00\x42\x03'), # get engine load
   bytearray(b'\x06\x00\x01\x02\x00\x62\x03'), # get injection time
   bytearray(b'\x06\x00\x01\x01\x00\x22\x03'), # get ac drive/switch
   bytearray(b'\x06\x00\x01\x01\x00\x29\x03'), # get O2 status
   bytearray(b'\x06\x00\x01\x01\x01\x90\x03'), # get fuel pump relay status
   bytearray(b'\x06\x00\x01\x01\x01\xb0\x03'), # get adsorber valve status
#    bytearray(b'\x04\x00\x04\x0e\x03'),
#    bytearray(b'\x04\x00\x04\x1f\x03'),
#    bytearray(b'\x04\x00\x04\x21\x03'),
   bytearray(b'\x03\x00\x06\x03'), # end session
]

for request in test_requests:
    print('---- send_packet ----')

    send_packet(request)

    time.sleep(0.05)

    print('---- recv_packet ----')

    recv_packet()

    time.sleep(0.02)
