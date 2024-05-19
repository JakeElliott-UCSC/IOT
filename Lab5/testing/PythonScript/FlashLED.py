import gpiozero
import time

led = gpiozero.LED(17)


while true:
    led.on()
    sleep(1)
    led.off()
    sleep(1)