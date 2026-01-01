import serial
import time
import binascii

def main():
    try:
        ser = serial.Serial("/dev/ttyACM1", 2000000, timeout=1)
        
        # Reset sequence (Standard ESP32 DTR/RTS logic)
        print("Resetting board...")
        ser.dtr = False
        ser.rts = True  # Pull EN low
        time.sleep(0.1)
        ser.rts = False # Release EN
        ser.dtr = False 
        
        print("Listening on /dev/ttyACM1...")
        while True:
            data = ser.read(1024)
            if data:
                print(binascii.hexlify(data).decode('utf-8'))
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
