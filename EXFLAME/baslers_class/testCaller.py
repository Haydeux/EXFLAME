import subprocess
import time
import signal


def start(executable_file):
    return subprocess.Popen(
        executable_file,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

def read(process):
    return process.stdout.readline().decode("utf-8").strip()


def write(process):
    process.send_signal(signal.SIGINT)

def terminate(process):
    process.stdin.close()
    process.terminate()
    process.wait(timeout=0.2)


process = start("./baslers_class/baslers_class")
time.sleep(10)
print("waited")
print(read(process)) 
time.sleep(5)

print("closing")
write(process)


