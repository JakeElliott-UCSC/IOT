import gpiozero
import time

led = gpiozero.LED(17)

x = 0
#time_delta = 0.25

while x < 20:
    led.on()
    time.sleep(0.25)
    led.off()
    time.sleep(0.25)
    x = x+1