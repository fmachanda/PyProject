import os
from os import listdir
from os.path import isfile, join
import numpy as np

#ONLY FLETCHER
import platform
if platform.system()=='Darwin':
    file_path='/Users/fletcher/Documents/PyProject'
elif platform.system()=='Windows':
    file_path=r'C:\Users\fmach\Documents\PyProject'
os.chdir(file_path)


import image_decoder as decoder

class xdata:

    def __init__(self,npy_path=None):

        if npy_path == None:
            self.data=np.empty((0,decoder.height,decoder.width,3))
        else:
            self.data=np.load(npy_path)

    def add_dir(self, my_dir:str):

        x_list=[]

        files = [f for f in listdir(my_dir) if isfile(join(my_dir, f))]

        for f in sorted(files):
            print(f)
            x=decoder.decode_image(join(my_dir, f))
            x=np.expand_dims(x,axis=0)
            x_list.append(x)
        
        added_data=np.concatenate(x_list,axis=0)
        self.data=np.concatenate((self.data,added_data),axis=0)
        self.data=(np.rint(self.data)).astype(int)

    def add_data(self, file_name:str):

        x=decoder.decode_image(file_name)
        x=np.expand_dims(x,axis=0)

        self.data=np.concatenate((self.data,x),axis=0)
        self.data=(np.rint(self.data)).astype(int)

    def save(self,name:str):

        np.save(name,self.data)

    def reset(self):
        
        self.data=np.empty((0,decoder.height,decoder.width,3))