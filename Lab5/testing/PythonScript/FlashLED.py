import gpiozero
import time

led = gpiozero.LED(17)

# MorseArray = [0] * 123

# # 'a' = 97 in decimal
# MorseArray[ord('a') - ord('0')] = 210

# print(MorseArray[ord('a') - ord('0')])

MorseArray = {
    ' ':[' '],
    'a':['.','-'],
    'b':['-','.','.','.'],
    'c':['-','.','-','.'],
    'd':['-','.','.'],
    'e':['.'],
    'f':['.','.','-','.'],
    'g':['-','-','.'],
    'h':['.','.','.','.'],
    'i':['.','.'],
    'j':['.','-','-','-'],
    'k':['-','.','-'],
    'l':['.','-','.','.'],
    'm':['-','-'],
    'n':['-','.'],
    'o':['-','-','-'],
    'p':['.','-','-','.'],
    'q':['-','-','.','-'],
    'r':['.','-','.'],
    's':['.','.','.'],
    't':['-'],
    'u':['.','.','-'],
    'v':['.','.','.','-'],
    'w':['.','-','-'],
    'x':['-','.','.','-'],
    'y':['-','.','-','-'],
    'z':['-','-','.','.']
              }



x = 0
time_delta = 0.2


for i in "hello world":
    for j in MorseArray[i]:
        print(j,end='')
    print(' ',end='')

print()


while x < 20:
    led.on()
    time.sleep(time_delta)
    led.off()
    time.sleep(time_delta)
    x = x+1