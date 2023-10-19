# Use this script to acquire stream of images into dest_path
import socket
import shutil,os

#ONLY FLETCHER
import platform
if platform.system()=='Darwin':
    file_path=r'/Users/fletcher/Documents/PyProject'
elif platform.system()=='Windows':
    file_path=r'C:\Users\fmach\Documents\PyProject'
os.chdir(file_path)

import find_xp as xp
import struct
import time

from os import listdir
from os.path import isfile, join

#ONLY FLETCHER
if platform.system()=='Darwin':
    dest_path=r'/Users/fletcher/Documents/PyProject/test_images'
    xp_path=r'/Users/fletcher/Documents/FS_Master/FS/Output'
elif platform.system()=='Windows':
    dest_path=r'C:\Users\fmach\Documents\PyProject\test_images'
    xp_path=r'C:\X-Plane 12\Output\screenshots'
os.chdir(file_path)

delay=1
previousFileList=[f for f in listdir(xp_path) if isfile(join(xp_path, f))]

cmd=b'CMND'
change_view=struct.pack("<4sx400s", cmd, b'sim/view/forward_with_nothing')
take_screenshot=struct.pack("<4sx400s", cmd, b'sim/operation/screenshot')
sock.sendto(change_view, (X_PLANE_IP, UDP_PORT))
sock.sendto(take_screenshot, (X_PLANE_IP, UDP_PORT))

time.sleep(delay)
    
newFileList = [f for f in listdir(xp_path) if isfile(join(xp_path, f))]

fileDiff = [x for x in newFileList if x not in previousFileList]

previousFileList = newFileList

if len(fileDiff) != 0:
    os.chdir(xp_path)
    for f in fileDiff:
        shutil.move(f, dest_path)
    print(fileDiff)

time.sleep(delay)