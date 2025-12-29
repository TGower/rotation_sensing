import struct

with open("/home/t/esp/rotation_sensing/receiver/test_pipeline.bin", "rb") as f:
    for i in range(10):
        chunk = f.read(131)
        if len(chunk) < 131:
            break
        magic, local_ts, sender_ts = struct.unpack_from("<Iqq", chunk)
        print(f"Packet {i}: Magic=0x{magic:X}, LocalTS={local_ts}, SenderTS={sender_ts}")
