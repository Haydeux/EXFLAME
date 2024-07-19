import subprocess
import time
import signal

import cv2 as cv
import os


def start_process(executable_file):
    return subprocess.Popen(
        executable_file,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

def read_process(process):
    return process.stdout.readline().decode("utf-8").strip()

def stop_process(process):
    process.send_signal(signal.SIGINT)

def terminate(process):
    process.stdin.close()
    process.terminate()
    process.wait(timeout=0.2)


#process = start_process("/home/geri/Documents/EXFLAME/Git_clone/EXFLAME/2024/detection_connect_test.py")

process = subprocess.Popen(['/usr/bin/python3', '/home/geri/Documents/EXFLAME/Git_clone/EXFLAME/2024/detection_connect_test.py'])


