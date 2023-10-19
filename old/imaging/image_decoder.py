import tensorflow as tf
import os
import numpy as np

#ONLY FLETCHER
import platform
if platform.system()=='Darwin':
    file_path=r'/Users/fletcher/Documents/PyProject'
elif platform.system()=='Windows':
    file_path=r'C:\Users\fmach\Documents\PyProject'
os.chdir(file_path)

width,height=256,144

def decode_image(img_name:str):
    void,extension=os.path.splitext(img_name)
    if extension=='.png':
        raw=tf.io.read_file(img_name)
        decoded_image=tf.image.decode_png(raw, channels=3)
        resized_image=tf.image.resize(decoded_image,(height,width))
        final_image=(np.rint(resized_image)).astype(int)
        return final_image
    return None