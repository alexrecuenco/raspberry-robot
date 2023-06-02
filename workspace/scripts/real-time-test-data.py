import random
from sys import stderr
from time import sleep

for i in range(300):
    sleep(0.05)
    print(random.random(), ", ", random.random() + 1, flush=True)
print("done", file=stderr)
