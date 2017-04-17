from subprocess import check_output
from datetime import datetime
import RPi.GPIO as GPIO
import os

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(16,GPIO.OUT)
GPIO.output(16,GPIO.LOW)

def get_pid(name):
    return map(int,check_output(["pidof",name]).split())
lastTime = datetime.now()

print get_pid("python")
while True:
    currrentTime = datetime.now()
    timeDiff = currrentTime - lastTime;

    if len(get_pid("python")) > 1:
        GPIO.output(16,GPIO.HIGH)
        lastTime = datetime.now()
    else:
        GPIO.output(16,GPIO.LOW)
        print "WARN: Code Not Running Rebooting in 60 seconds"
        if timeDiff.seconds > 60:
            print "Restarting System"
            #os.system('reboot')
