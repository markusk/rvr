#!/usr/bin/python3
# coding=utf-8
# -*- coding: utf-8 -*-

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
    print('Echo response: ', echo_response)
    RVRisOn = True


def main():
    print("Waking up RVR...")
    rvr.wake()
    # Give RVR time to wake up
    time.sleep(2)
    print("done.")


    # ping RVR
    rvr.echo(
        data=[4, 2],
        handler=echo_handler,
        target=SpheroRvrTargets.primary.value
    )
    # Give RVR time to respond
    time.sleep(1)


    print("Closing connection to RVR...")
    rvr.close()
    print("done.")


if __name__ == '__main__':
    main()
