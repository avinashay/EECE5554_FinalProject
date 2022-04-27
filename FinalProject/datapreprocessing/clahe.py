import cv2
import numpy as np
import os

for i in range(5252):
    filename = './bagtoimg/big/image_1/'+'0'*(6-len(str(i))) + str(i)+'.png'
    image = cv2.imread(filename)
    image_bw = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit = 3.0, tileGridSize=(8,8))
    cl_img = clahe.apply(image_bw)
    cv2.imwrite(os.path.join('./bagtoimg/big/clahe/image_1/', '0'*(6-len(str(i))) + str(i) + ".png"), cl_img)
    print(i)
    #break