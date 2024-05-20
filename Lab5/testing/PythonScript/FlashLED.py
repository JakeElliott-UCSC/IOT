import gpiozero
import time

led = gpiozero.LED(17)

# MorseArray = [0] * 123

# # 'a' = 97 in decimal
# MorseArray[ord('a') - ord('0')] = 210

# print(MorseArray[ord('a') - ord('0')])

MorseArray = {
    'a':210,
    'b':['-','.','.','.']
              }




print(MorseArray['b'])

for i in MorseArray['b']:
    print(i,end='')


x = 0
time_delta = 0.2






while x < 20:
    led.on()
    time.sleep(time_delta)
    led.off()
    time.sleep(time_delta)
    x = x+1