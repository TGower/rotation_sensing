import struct
import sys

PACKET_FMT = "<IqqhhhBhq512s"
PACKET_SIZE = struct.calcsize(PACKET_FMT)
MAGIC = 0xDEADBEEF

def main():
    try:
        with open("working_log.bin", "rb") as f:
            content = f.read()
            
        offset = 0
        packet_count = 0
        
        while offset < len(content) and packet_count < 1:  # Check first packet only for structure
            idx = content.find(struct.pack("<I", MAGIC), offset)
            if idx == -1: break
            if idx + PACKET_SIZE > len(content): break
                
            data = content[idx : idx + PACKET_SIZE]
            unpacked = struct.unpack(PACKET_FMT, data)
            
            csi_len = unpacked[7]
            csi_bytes = unpacked[9][:csi_len]
            
            # Assume int8 pairs (Real, Imag)
            # 128 bytes = 64 subcarriers
            subcarriers = []
            zeros = []
            
            print(f"Packet {packet_count+1}: {csi_len} bytes CSI")
            
            for i in range(0, csi_len, 2):
                if i+1 >= len(csi_bytes): break
                r = csi_bytes[i]
                imag = csi_bytes[i+1] # Avoid 'i' variable name conflict
                
                # Convert to signed int8 if needed, but Python bytes are 0-255. 
                # If 0, it's 0 in either signed/unsigned.
                
                idx_sub = i // 2
                if r == 0 and imag == 0:
                    zeros.append(idx_sub)
                
                # To print signed values:
                if r > 127: r -= 256
                if imag > 127: imag -= 256
                subcarriers.append((r, imag))
            
            print(f"Total Subcarriers: {len(subcarriers)}")
            print(f"Zero indices: {zeros}")
            
            offset = idx + PACKET_SIZE
            packet_count += 1
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
