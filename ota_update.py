import serial
import time
import sys
import struct
import argparse

SERIAL_START_BYTE = 0xAA
SERIAL_CMD_START = 0xA0
SERIAL_CMD_DATA = 0xA1
SERIAL_CMD_END = 0xA2
SERIAL_RESP_ACK = 0x06
SERIAL_RESP_NACK = 0x15

CHUNK_SIZE = 200

def calculate_checksum(cmd_type, payload):
    cs = cmd_type
    len_val = len(payload)
    cs ^= (len_val & 0xFF)
    cs ^= ((len_val >> 8) & 0xFF)
    for b in payload:
        cs ^= b
    return cs

def send_command(ser, cmd_type, payload):
    cs = calculate_checksum(cmd_type, payload)
    length = len(payload)

    pkt = struct.pack('<BBH', SERIAL_START_BYTE, cmd_type, length)
    pkt += payload
    pkt += struct.pack('B', cs)

    ser.write(pkt)

    # Wait for response
    # Response: AA CMD STATUS
    start = time.time()
    while time.time() - start < 2.0: # 2 sec timeout
        if ser.in_waiting >= 3:
            resp = ser.read(3)
            if resp[0] == SERIAL_START_BYTE and resp[1] == cmd_type:
                return resp[2] == SERIAL_RESP_ACK
    return False

def mac_to_bytes(mac_str):
    return bytes.fromhex(mac_str.replace(':', ''))

def main():
    parser = argparse.ArgumentParser(description='ESP-NOW OTA Updater')
    parser.add_argument('port', help='Serial Port of OTA Server')
    parser.add_argument('file', help='Firmware Bin File')
    parser.add_argument('mac', help='Target MAC Address (e.g. AA:BB:CC:DD:EE:FF)')

    args = parser.parse_args()

    mac_bytes = mac_to_bytes(args.mac)
    if len(mac_bytes) != 6:
        print("Invalid MAC address")
        return

    try:
        ser = serial.Serial(args.port, 115200, timeout=0.1)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    print(f"Opening {args.file}...")
    with open(args.file, 'rb') as f:
        data = f.read()

    total_size = len(data)
    print(f"Total Size: {total_size} bytes")

    # 1. Send START
    print("Sending START...")
    payload = struct.pack('<I', total_size) + mac_bytes
    if not send_command(ser, SERIAL_CMD_START, payload):
        print("START failed or timed out. Check connection/target.")
        return
    print("START ACKed.")

    # 2. Send DATA
    num_chunks = (total_size + CHUNK_SIZE - 1) // CHUNK_SIZE
    seq = 0

    for i in range(num_chunks):
        chunk = data[i*CHUNK_SIZE : (i+1)*CHUNK_SIZE]

        # Payload: Seq(2), Len(2), Data(...)
        payload = struct.pack('<HH', seq, len(chunk)) + chunk

        retries = 5
        success = False
        while retries > 0:
            if send_command(ser, SERIAL_CMD_DATA, payload):
                success = True
                break
            print(f"Retry Seq {seq}...")
            retries -= 1

        if not success:
            print(f"Failed to send Seq {seq} after retries. Aborting.")
            return

        print(f"\rProgress: {i+1}/{num_chunks} ({int((i+1)/num_chunks*100)}%)", end='')
        seq += 1

    print("\nSending END...")
    if send_command(ser, SERIAL_CMD_END, b''):
        print("OTA Complete! Target should be rebooting.")
    else:
        print("END NACKed or timed out (Target might have rebooted already).")

    ser.close()

if __name__ == '__main__':
    main()
