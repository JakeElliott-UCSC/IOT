import gpiozero
import time

led = gpiozero.LED(17)

x = 0
while x < 20:
    led.on()
    sleep(1)
    led.off()
    sleep(1)
    x = x+1