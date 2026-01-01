import struct
import sys

# Structure: Header(37) + CSI(94)
PACKET_FMT = "<IqqhhhBhq94s"
PACKET_SIZE = struct.calcsize(PACKET_FMT)
MAGIC = 0xDEADBEEF

def main():
    try:
        with open("optimized_v3.bin", "rb") as f:
            content = f.read()
            
        offset = 0
        packet_count = 0
        
        while offset < len(content) and packet_count < 1:
            idx = content.find(struct.pack("<I", MAGIC), offset)
            if idx == -1: break
            
            data = content[idx : idx + PACKET_SIZE]
            unpacked = struct.unpack(PACKET_FMT, data)
            
            csi_bytes = unpacked[9] # 94 bytes
            
            zeros = []
            
            print(f"Packet {packet_count+1}: 94 bytes CSI (Optimized v3)")
            
            for i in range(0, 94, 2):
                r = csi_bytes[i]
                imag = csi_bytes[i+1]
                
                idx_sub = i // 2
                if r == 0 and imag == 0:
                    zeros.append(idx_sub)
            
            print(f"Zero indices in compacted array (0-46): {zeros}")
            
            # Map back to original indices for clarity
            # Compacted 0-29 -> Original 1-30
            # Compacted 30-52 -> Original 41-63
            
            original_zeros = []
            for z in zeros:
                if z < 30:
                    original = z + 1
                else:
                    original = 41 + (z - 30)
                original_zeros.append(original)
                
            print(f"Corresponding Original Indices: {original_zeros}")
            
            offset = idx + PACKET_SIZE
            packet_count += 1
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
