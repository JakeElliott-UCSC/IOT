import gpiozero
import time

led = gpiozero.LED(17)

x = 0
while x < 20:
    led.on()
    time.sleep(1)
    led.off()
    time.sleep(1)
    x = x+1