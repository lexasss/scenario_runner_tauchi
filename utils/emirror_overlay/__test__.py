import subprocess
import sys

from time import sleep

print('Start')

subprocess.Popen([sys.executable, "__main__.py"])

for i in range(5):
    sleep(1)
    print(i)
    
print('Exit')