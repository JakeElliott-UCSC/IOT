import gpiozero
import time

led = gpiozero.LED(17)

x = 0
time_delta = 0.2

while x < 20:
    led.on()
    time.sleep(time_delta)
    led.off()
    time.sleep(time_delta)
    x = x+1