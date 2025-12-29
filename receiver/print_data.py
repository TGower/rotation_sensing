import struct
import sys
import argparse

# Structure
# Magic(4), LocalTS(8), SenderTS(8), X(2), Y(2), Z(2), RSSI(1), CSILen(2), CSITS(8), CSIData(94)
PACKET_FMT = "<IqqhhhBhq94s"
PACKET_SIZE = struct.calcsize(PACKET_FMT)
MAGIC = 0xDEADBEEF

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="Input file", nargs="?", default="optimized_log.bin")
    args = parser.parse_args()

    try:
        with open(args.file, "rb") as f:
            content = f.read()
            
        offset = 0
        count = 0
        while offset < len(content) and count < 5:
            # Find magic
            idx = content.find(struct.pack("<I", MAGIC), offset)
            if idx == -1:
                break
                
            if idx + PACKET_SIZE > len(content):
                break
                
            data = content[idx : idx + PACKET_SIZE]
            unpacked = struct.unpack(PACKET_FMT, data)
            
            magic = unpacked[0]
            local_ts = unpacked[1]
            sender_ts = unpacked[2]
            x, y, z = unpacked[3], unpacked[4], unpacked[5]
            rssi = unpacked[6]
            csi_len = unpacked[7]
            csi_ts = unpacked[8]
            # csi_data is unpacked[9]
            
            print(f"Packet {count+1}:")
            print(f"  Local TS:  {local_ts/1000000:.6f} s")
            print(f"  Sender TS: {sender_ts/1000000:.6f} s")
            print(f"  TMAG Data: X={x}, Y={y}, Z={z}")
            print(f"  RSSI:      {rssi} dBm")
            print(f"  CSI Len:   {csi_len} bytes")
            
            # Print valid CSI data
            valid_csi = unpacked[9][:csi_len]
            print(f"  CSI Data:  {valid_csi.hex()}")
            print("-" * 40)
            
            offset = idx + PACKET_SIZE
            count += 1
            
    except FileNotFoundError:
        print("Error: working_log.bin not found")
    except Exception as e:
        print(f"Error parsing file: {e}")

if __name__ == "__main__":
    main()
