import gpiozero
import time
import sys

led = gpiozero.LED(17)

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
    'z':['-','-','.','.'],

    'A':['.','-'],
    'B':['-','.','.','.'],
    'C':['-','.','-','.'],
    'D':['-','.','.'],
    'E':['.'],
    'F':['.','.','-','.'],
    'G':['-','-','.'],
    'H':['.','.','.','.'],
    'I':['.','.'],
    'J':['.','-','-','-'],
    'K':['-','.','-'],
    'L':['.','-','.','.'],
    'M':['-','-'],
    'N':['-','.'],
    'O':['-','-','-'],
    'P':['.','-','-','.'],
    'Q':['-','-','.','-'],
    'R':['.','-','.'],
    'S':['.','.','.'],
    'T':['-'],
    'U':['.','.','-'],
    'V':['.','.','.','-'],
    'W':['.','-','-'],
    'X':['-','.','.','-'],
    'Y':['-','.','-','-'],
    'Z':['-','-','.','.'],

    '1':['.','-','-','-','-'],
    '2':['.','.','-','-','-'],
    '3':['.','.','.','-','-'],
    '4':['.','.','.','.','-'],
    '5':['.','.','.','.','.'],
    '6':['-','.','.','.','.'],
    '7':['-','-','.','.','.'],
    '8':['-','-','-','.','.'],
    '9':['-','-','-','-','.'],
    '0':['-','-','-','-','-']
              }


x = 0
time_delta = 0.2










def printMorse(message):
    for i in message:
        for j in MorseArray[i]:
            print(j,end='',flush=True)
            if j == '.':
                led.off()
                time.sleep(time_delta)
                led.on()
                time.sleep(time_delta)
                led.off()
                time.sleep(time_delta*10)
            if j == '-':
                led.off()
                time.sleep(time_delta)
                led.on()
                time.sleep(time_delta*10)
                led.off()
                time.sleep(time_delta)
            if j == ' ':
                led.off()
                time.sleep(time_delta*20)
        print(' ',end='',flush=True)
    print()


print("from Python: " + sys.argv[1])
print("from Python: " + sys.argv[2])






printMorse("Hello World")
printMorse("ESP 32")
























while x < 20:
    led.on()
    time.sleep(time_delta)
    led.off()
    time.sleep(time_delta)
    x = x+1