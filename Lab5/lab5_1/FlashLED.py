import gpiozero
import time
import sys

led = gpiozero.LED(17)

MorseArray = {
    ' ':['/ '],
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
time_delta = 0.1









# time delta is 100 ticks in esp
# dot is up for 300 ticks
# dash is up for 800 ticks
def printMorse(message):
    # iterate over recieved text
    for i in message:
        # iterate over morse code version of each ascii character
        for j in MorseArray[i]:
            # print the morse character
            print(j,end='',flush=True)
            # flash the morse character
            if j == '.':
                led.off()
                time.sleep(time_delta) # 100 ticks / 4 main loops
                led.on()
                time.sleep(time_delta*0.75) # 75 ticks / 3 main loops
                led.off()
                time.sleep(time_delta*8) # 800 ticks / 32 main loops
            if j == '-':
                led.off()
                time.sleep(time_delta) # 100 ticks / 4 main loops
                led.on()
                time.sleep(time_delta*3) # 300 ticks / 12 main loops
                led.off()
                time.sleep(time_delta*6) # 600 ticks / 24 main loops
            if j == '/':
                led.off()
                time.sleep(time_delta*50) # 2000 ticks / 80 main loops
            if j == '5':
                print("placeholder")
        print(' ',end='',flush=True)
    print()


# runtime code

message = " ".join(sys.argv[2:])
# debugging
print(message)

# print the message  multiple times
for rep in range(int(sys.argv[1])):
    printMorse(message)
    time.sleep(time_delta * 50) # 5000 ticks / 200 main loops


