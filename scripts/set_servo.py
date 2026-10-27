#!/usr/bin/env python3
import sys
import os
import fcntl
import struct

def set_servo(angle):
    i2c_bus = "/dev/i2c-1"
    esp32_addr = 0x68

    print(f"Ustawianie serwa na: {angle}° (Adres ESP32: 0x{esp32_addr:02X})")
    try:
        fd = os.open(i2c_bus, os.O_RDWR)
        fcntl.ioctl(fd, 0x0703, esp32_addr) # I2C_SLAVE

        # Ramka RPI ---> ESP32: 3 x int32_t little-endian (right_speed, left_speed, servo_pos)
        frame = struct.pack('<iii', 0, 0, int(angle))
        bytes_sent = os.write(fd, frame)
        os.close(fd)
        print(f"Sukces! Wysłano {bytes_sent} bajtów do ESP32 (serwo = {angle}°)")
    except PermissionError:
        print("Błąd uprawnień: uruchom skrypt z sudo (np. sudo python3 set_servo.py 60)")
    except OSError as e:
        print(f"Błąd I2C: [Errno {e.errno}] {os.strerror(e.errno)}")
    except Exception as e:
        print(f"Nieoczekiwany błąd: {e}")

if __name__ == "__main__":
    angle = 90 # domyślnie 90 st. (środek pomiędzy 50 a 130)
    if len(sys.argv) > 1:
        try:
            angle = int(sys.argv[1])
        except ValueError:
            print("Podaj kąt jako liczbę całkowitą (np. 60, 90, 120)")
            sys.exit(1)
    set_servo(angle)
