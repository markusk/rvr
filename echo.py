#!/usr/bin/python3
# coding=utf-8
# -*- coding: utf-8 -*-


""" This program checks if the Sphero RVR is awake and waits for this state until it is turned on """


import os
import sys
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './lib/')))

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import SpheroRvrTargets


RVRisOn = False


rvr = SpheroRvrObserver()


def echo_handler(echo_response):
    global RVRisOn
    RVRisOn = True
    print("\n++++++++++++++++")
    print("+ RVR woke up! +")
    print("++++++++++++++++\n")
    #print("RVR pings back with: ", echo_response)


def main():
    global RVRisOn

    while RVRisOn == False:
        print("Waking up RVR...")
        rvr.wake()
        # Give RVR time to wake up
        time.sleep(2)
        #print("done.")

        # ping RVR
        rvr.echo(
            data=[42],
            handler=echo_handler,
            target=SpheroRvrTargets.primary.value
        )
        # Give RVR time to respond
        time.sleep(1)

    # RVR woke up
    print("Closing connection to RVR...")
    rvr.close()
    print("done.")


if __name__ == '__main__':
    main()
