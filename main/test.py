import serial

import time

SERIAL_DEVICE = '/dev/ttyUSB0'

# linux ch341 module works very strange, on 8860 baud it sets real serial speed at half of requested, but works fine with ~8830-8850 baud rate
serial_port = serial.Serial(SERIAL_DEVICE, 8830, timeout=2)

def esp_reset():
    # Reset
    serial_port.setDTR(False)

    time.sleep(1)

    serial_port.setDTR(True)

def init_connection():
    # Boot (GPIO0)
    serial_port.setRTS(False)
    time.sleep(1)
    serial_port.setRTS(True)
    time.sleep(0.2)
    serial_port.setRTS(False)
    time.sleep(0.6)
    serial_port.setRTS(True)

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

#esp_reset()

#time.sleep(1)

serial_port.reset_input_buffer()

serial_port.reset_output_buffer()

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

# while True:
#     print('---- recv_packet ----')

#     recv_packet()

#     time.sleep(0.02)

#     print('---- send_packet ----')

#     send_packet(bytearray(b'\x03\x02\x09\x03'))

#     time.sleep(0.05)
