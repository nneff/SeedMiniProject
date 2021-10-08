import time
import smbus
###############################################################
DISTANCE_THRESH = 50
bus = smbus.SMBus(1)
address = 0x04

def writeByte(value):
    bus.write_byte(address, value)
    print("writeByte done", value, "  \n")
    return

def main():
    i = 1
    while i <= 4:
        writeByte(i)
        i += 1
        time.sleep(10)
    
if __name__ == "__main__":
    main()