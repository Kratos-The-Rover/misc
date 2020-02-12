#####################DEFAULT///////////////////////

import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy, cv_bridge, numpy
from sensor_msgs.msg import Image
#%matplotlib notebook

#//////////////////////////////////////////////

from mpl_toolkits.mplot3d import Axes3D
#from jupyterthemes import jtplot
#%config InlineBackend.figure_format ='retina'
#jtplot.style()

#///////////////////////////////////////////////

import math

##################////FUNCTIONS
rospy.init_node('science')
#####################################################################################
##Input
led_wavelength_ = [385,405,430,465,480,505,535,572,591,615,626,655,700,830,860,890,935,950]
led_angle = [30, 15, 60, 15,15, 30, 15, 16, 15, 15, 15, 16, 50, 38, 10, 16, 18, 15]
max_at_0 = [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
max_at_0 = 255
led_angle = 15
img_from_pi = "0.jpg"



def find_max():
    #led_angle = led_angle_in
    A = np.zeros([pixels(led_angle, 0), pixels(led_angle, 0)]) 
    f=1

    intensity = intensity_finder()
    A[pixels(led_angle,1)][pixels(led_angle,1)] = max_at_0
    for x in range(0,intensity.pixel_list_length):
        for i in range(pixels(led_angle,1)+1+int(intensity.pixel_list[x]), pixels(led_angle,1)+1+int(intensity.pixel_list[x+1])):
            f = f + 2
    
            m = 0 
            n = 0
            o = 0
            p = 0
    
            for m in range(0,f):
                #print("M = {}".format(m))
                A[i, i-m] = intensity.max_list[x]     
            for n in range(0,f):
                #print("M = {}".format(n))
                A[i-n, i - f + 1] = intensity.max_list[x]
            for o in range(0,f):
                #print("M = {}".format(o))
                A[i - f + 1, i - f + 1 + o] = intensity.max_list[x]
            for p in range(0,f):
                #print("M = {}".format(p))
                A[i - f + 1 + p, i] = intensity.max_list[x]


    delete = pixels(led_angle,0) - pixels(led_angle,1) - 1 - int(intensity.pixel_list[x]) 
    A = A.astype(int)


    A_sliced = A[delete:, delete:]
    A_sliced = A_sliced[:-delete, :-delete]
    
    #print(len(A))
    
    A_cropper = int(len(A_sliced)/2 - 514)
    if A_cropper > 0 :
        A_cropped = A_sliced[A_cropper:, A_cropper:]
        A_cropped = A_cropped[:-A_cropper, :-A_cropper]
        
        #plt.figure()
        plt.subplot(2,2,3)
        plt.imshow(A_cropped, cmap = "gray")
        plt.title("Cropped")
        #cv2.imshow(A_cropped, cmap = "gray")
        #cv2.waitKey(10000)
    else:
        #np.savetxt("Max", A_cropped)
        #plt.figure()
        plt.subplot(2,2,3)
        plt.imshow(A_sliced, cmap = "gray")
        plt.title("Sliced")
        A_cropped = A
        #cv2.imshow(A_sliced, cmap = "gray")
        #cv2.waitKey(10000)
    return A_cropped

def rad(angle):
    radian = (angle*math.pi)/180
    return radian

def pixels(ledangle, half):
    pixel_total = 3280 * math.cos(rad(45)) * math.tan(rad(ledangle))/math.tan(rad(31.1))
    pixel_total = int(pixel_total)
    if pixel_total % 2 == 0:
        pixel_total = pixel_total - 1
    pixel_half = int(pixel_total/2)
    if half == 0:
        return pixel_total
    else:
        return pixel_half
    
class intensity_finder:
    
    pixel_list_length=0
    pixel_list = np.zeros(1000)
    max_list = np.zeros(1000)
    
    
    for a in range(0,led_angle*4 + 2):    
        aby8 = a/8
        pixel_alpha = pixels(aby8,0)
        rev_pixel_alpha = pixels(led_angle,1) - pixel_alpha
        maxi = (max_at_0 * rev_pixel_alpha * math.tan(rad(led_angle/2)))/(pixels(led_angle,1) * math.tan(rad(led_angle/2)))
        maxi = int(maxi)
        max_list[pixel_list_length] = maxi
        pixel_list[pixel_list_length] = pixel_alpha
        pixel_list_length+=1
find_max()


reflected_ori = cv2.imread(img_from_pi, 1)




###SquareCroppingSymmetric
reflected_squarer = int((reflected.shape[1] - reflected.shape[0])/2)
reflected_squared = reflected[:, reflected_squarer:]
reflected_squared = reflected_squared[:, :-reflected_squarer]

### Cropping
reflected_cropper = int(len(reflected_squared)/2 - 514)
reflected_cropped = reflected_squared[reflected_cropper:, reflected_cropper:]
reflected_cropped = reflected_cropped[:-reflected_cropper+1, :-reflected_cropper+1]

plt.subplot(2,2,2)
plt.imshow(reflected_cropped, cmap = "gray")
plt.title("Original")

###divided
divided_img = np.divide(find_max(), reflected_cropped)
#plt.figure()
plt.subplot(2,2,4)
plt.imshow(divided_img, cmap="gray")
plt.title("Divided image")

divided_img *= 100
#print(divided_img)
while not rospy.is_shutdown():
	science_image=rospy.Publisher('Science_image_data',Image,queue_size=1)
	science_image.publish(divided_img)
