
import serial
import struct
import time
import argparse
import random

# Protocol
# Control: [START] [TYPE] [MAGIC] [THROTTLE:2] [VX:4] [VY:4] [CHECKSUM]
START_BYTE = 0xAA
RECV_START_BYTE = 0xAB

APP_PACKET_TYPE_CONTROL = 0x10
APP_PACKET_TYPE_CONFIG  = 0x20
APP_PACKET_TYPE_STATS   = 0x30
APP_PROTOCOL_MAGIC = 164



def parse_hex_string(hex_str):
    try:
        data = bytes.fromhex(hex_str)
        return data
    except ValueError:
        return None

def create_control_packet(throttle, vx, vy):
    # Payload: Type(B) + Magic(B) + Throttle(H) + Vx(f) + Vy(f)
    payload = struct.pack('<BBHff', APP_PACKET_TYPE_CONTROL, APP_PROTOCOL_MAGIC, throttle, vx, vy)
    
    checksum = 0
    for b in payload:
        checksum ^= b
        
    return bytes([START_BYTE]) + payload + bytes([checksum])

def create_config_packet(source, step_lag, step_window):
    payload = struct.pack(
        '<BBBBBBHHffHHfBB',
        APP_PACKET_TYPE_CONFIG,
        APP_PROTOCOL_MAGIC,
        13,
        9,
        12,
        source,
        step_lag,
        step_window,
        2.0,
        4.0,
        1000,
        20,
        0.0,
        1,
        3,
    )
    
    checksum = 0
    for b in payload:
        checksum ^= b
        
    return bytes([START_BYTE]) + payload + bytes([checksum])

def main():
    parser = argparse.ArgumentParser(description='Sender Test Script')
    parser.add_argument('--port', default='/dev/ttyACM1', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        ser.dtr = False
        ser.rts = False
        print(f"Opened {args.port} at {args.baud}")
        
        # Reset the board?
        # ser.dtr = True
        # ser.rts = True
        # time.sleep(0.1)
        # ser.dtr = False
        # ser.rts = False
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    print("Sending packets... Press 'c' to send Config.")
    
    # Config State
    cfg_source = 0 # CSI
    cfg_lag = 5
    cfg_win = 5
    
    # Read Buffer
    rx_buf = bytes()
    loop_count = 0
    
    try:
        while True:
            # Check for user input (simplistic, non-blocking check would be better but stdio is hard in loop)
            # We'll just rely on logic for now or random updates?
            # Let's just alternate or random for testing?
            # Or use a separate thread for reading keyboard.
            # For this test, let's just send Control continuously.
            
            # TODO: Interactive input
            
            # Generate random control data
            throttle = 100 + int(100 * (0.5 + 0.5 * random.random())) # 100-200
            vx = random.uniform(-1.0, 1.0)
            vy = random.uniform(-1.0, 1.0)
            
            # Occasionally send a Config packet (e.g. every 100 iterations ~ 10s)
            # Or just send control.
            
            packet = create_control_packet(throttle, vx, vy)
            ser.write(packet)
            loop_count += 1
            if loop_count % 100 == 0:
                print(f"Loop {loop_count}: InWaiting={ser.in_waiting}")
            
            # print(f"Sent Control: Throttle={throttle}")
            
            # Read Back
            # Read Back
            while ser.in_waiting:
                try:
                    line = ser.readline().decode(errors='ignore').strip()
                    if not line: continue
                    
                    if "STATS_DATA:" in line:
                        print(f"RAW_STATS: {line}") # Debug: Show we saw it
                        
                         # Found Data
                        parts = line.split("STATS_DATA:")

                        if len(parts) > 1:
                            raw_val = parts[1].strip()
                            # Split on ESC to remove color codes trailing
                            if '\x1b' in raw_val:
                                raw_val = raw_val.split('\x1b')[0]
                            
                            hex_str = "".join([c for c in raw_val if c in "0123456789ABCDEFabcdef"])
                            
                            rx_buf = parse_hex_string(hex_str)
                            
                            if rx_buf and len(rx_buf) >= 24:
                                 # Validate Checksum
                                calc_sum = 0
                                payload = rx_buf[1:-1]
                                for x in payload: calc_sum ^= x
                                
                                if calc_sum == rx_buf[-1]:
                                    # Parse Stats
                                    ptype = payload[0]
                                    if ptype == APP_PACKET_TYPE_STATS and len(payload) == struct.calcsize('<BBffiBfffI') and payload[1] == APP_PROTOCOL_MAGIC:
                                        _, _, rssi_mean, rssi_var, pkts, last_rssi, rotation_rate, vx, vy, autocorr = struct.unpack('<BBffiBfffI', payload)
                                        if last_rssi > 127:
                                            last_rssi -= 256
                                        print(f"STATS: Pkts={pkts} RSSI={last_rssi} Rate={rotation_rate:.2f}Hz Mean={rssi_mean:.1f} Var={rssi_var:.1f} Vec=({vx:.2f},{vy:.2f}) Autocorr={autocorr}us")
                                else:
                                    print(f"ERR: Stats checksum fail. Calc={calc_sum:02X} Recv={rx_buf[-1]:02X}")
                            else:
                                print(f"ERR: RX Buf short or None. Len={len(rx_buf) if rx_buf else 0} Hex={hex_str}")
                    else:
                        # Print other logs to act as monitor
                        print(f"DEV: {line}")
                        
                except Exception as e:
                    pass




                        
            time.sleep(0.1) # 10Hz
            
    except KeyboardInterrupt:
        # Prompt for Config on Exit?
        print("\nSending Config Update before exit...")
        # Swap Source
        cfg_source = 1 if cfg_source == 0 else 0
        print(f"Swapping Source to {cfg_source}")
        pkt = create_config_packet(cfg_source, 10, 10)
        ser.write(pkt)
        time.sleep(0.5)
        print("Done.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
