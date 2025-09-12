from machine import Pin
from utime import sleep

pin = Pin("LED", Pin.OUT)
blink = 0.1
print("LED starts flashing...")
while True:
    try:
        pin.toggle()
        sleep(blink)
    except KeyboardInterrupt:
        break
pin.off()
print("Finished.")
 # fake change to test github integration