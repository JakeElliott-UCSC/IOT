import gpiozero
import time

led = gpiozero.LED(17)

x = 0
while x < 20:
    led.on()
    gpiozero.sleep(1)
    led.off()
    gpiozero.sleep(1)
    x = x+1