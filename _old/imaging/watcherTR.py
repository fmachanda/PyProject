# Use this script to move new manual images to dest_path
import shutil,os
from os import listdir
from os.path import isfile, join
import time

# ONLY FLETCHER
import platform
if platform.system()=='Darwin':
    dest_path=r'/Users/fletcher/Documents/PyProject/test_images'
    xp_path=r'/Users/fletcher/Documents/FS_Master/FS/Output'
    file_path=r'/Users/fletcher/Documents/PyProject'
elif platform.system()=='Windows':
    file_path=r'C:\Users\fmach\Documents\PyProject'
    dest_path=r'C:\Users\fmach\Documents\PyProject\test_images'
    xp_path=r'C:\X-Plane 12\Output\screenshots'
os.chdir(file_path)

#function to return files in a directory
def fileInDirectory(my_dir: str):
    onlyfiles = [f for f in listdir(my_dir) if isfile(join(my_dir, f))]
    return(onlyfiles)

#function comparing two lists
def listComparison(OriginalList: list, NewList: list):
    differencesList = [x for x in NewList if x not in OriginalList] #Note if files get deleted, this will not highlight them
    return(differencesList)

def doThingsWithNewFiles(fileDiff: list):
    os.chdir(xp_path)
    for f in fileDiff:
        shutil.move(f, dest_path)
    print(fileDiff)

def fileWatcher(my_dir: str, pollTime: int):
    while True:
        if 'watching' not in locals(): #Check if this is the first time the function has run
            previousFileList = fileInDirectory(my_dir)
            watching = 1
        
        time.sleep(pollTime)
        
        newFileList = fileInDirectory(my_dir)
        
        fileDiff = listComparison(previousFileList, newFileList)
        
        previousFileList = newFileList
        if len(fileDiff) == 0: continue
        doThingsWithNewFiles(fileDiff)

fileWatcher(xp_path,5)

#   %run C:\Users\fmach\Documents\PyProject\watcher.py