#!/usr/bin/python
# coding=utf-8

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
BUZZER = 13
GPIO.setup(BUZZER, GPIO.OUT)

def buzz(noteFreq, duration):
    halveWaveTime = 1 / (noteFreq * 2 )
    waves = int(duration * noteFreq)
    for i in range(waves):
       GPIO.output(BUZZER, True)
       time.sleep(halveWaveTime)
       GPIO.output(BUZZER, False)
       time.sleep(halveWaveTime)

def play():
    t=0
    dur=0.09
    # the notes have these frequencies: https://pages.mtu.edu/~suits/notefreqs.html
    # Bruder Jakob
    # notes=[262,294,330,262,262,294,330,262,330,349,392,330,349,392,392,440,392,349,330,262,392,440,392,349,330,262,262,196,262,262,196,262]
    # duration=[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,1,0.5,0.5,1,0.25,0.25,0.25,0.25,0.5,0.5,0.25,0.25,0.25,0.25,0.5,0.5,0.5,0.5,1,0.5,0.5,1]
    notes    = [392,   392, 392,   1,  294,  392,  523,    1, 262]
    duration = [dur,   dur, dur, 0.3,  dur,  dur,  dur,  0.5, dur]
    for n in notes:
        buzz(n, duration[t])
        # pause between each note
        time.sleep(duration[t] *0.4)
        t+=1

play()
