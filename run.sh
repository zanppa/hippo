#!/bin/sh
tty=`tty`
python -u hippo_pigpio.py | tee $tty | socat -d -d stdin pty,raw,echo=0
