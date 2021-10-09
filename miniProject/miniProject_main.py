#Nash Neff
#SEED Lab computer Visionb
#9/27/2021
##############################################################
import cv2
import numpy as np
import math as m
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import smbus
###############################################################
DISTANCE_THRESH = 80
bus = smbus.SMBus(1)
address = 0x04

def writeByte(value):
    bus.write_byte(address, value)
    print("writeByte done \n")
    return

def find_target():

    camera = PiCamera()
    camera.resolution = (1280, 720)
    rawCapture = PiRGBArray(camera)
    
    time.sleep(.1)
    
    try:
        camera.capture(rawCapture, format='bgr')
        img = rawCapture.array
        camera.close()
    except:
        print("Failed to capture image \n")
    #get width and height to determine quadrant   
    IMG_HEIGHT, IMG_WIDTH, _ = img.shape
    
    #convert image to grayscale for binary operations
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #adaptive threshold to combat lighting
    adap_thresh_img = cv2.adaptiveThreshold(gray_img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, blockSize=75, C=5)
    
    #open and close image to get better tracking
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    adap_thresh_img = cv2.morphologyEx(adap_thresh_img, cv2.MORPH_OPEN, kernel)
    adap_thresh_img = cv2.morphologyEx(adap_thresh_img, cv2.MORPH_CLOSE, kernel)
    
    #find circles in image
    num_labels, labels_img, stats, centroids = cv2.connectedComponentsWithStats(adap_thresh_img)
    
    #invert image and find circles
    thresh_img_inv = cv2.bitwise_not(adap_thresh_img)
    num_labels_inv, labels_img_inv, stats_inv, centroids_inv = cv2.connectedComponentsWithStats(thresh_img_inv)
    
    #For each blob, check if there is another blob with the same center, and a smaller overall area, and a reasonable area
    #if all true, place marker and return quadrant location
    marker_img = cv2.cvtColor(adap_thresh_img, cv2.COLOR_GRAY2BGR)
    #cv2.imshow("test", marker_img)
    #cv2.waitKey(0)
    found_circle = False
    quadrant = 0
    marker_list = []
    for stat, centroid in zip(stats, centroids):
        x = stat[cv2.CC_STAT_LEFT]
        y = stat[cv2.CC_STAT_TOP]
        w = stat[cv2.CC_STAT_WIDTH]
        h = stat[cv2.CC_STAT_HEIGHT]
        area = stat[cv2.CC_STAT_AREA]
        for stat_inv, centroid_inv in zip(stats_inv, centroids_inv):
            x_inv = stat_inv[cv2.CC_STAT_LEFT]
            y_inv = stat_inv[cv2.CC_STAT_TOP]
            w_inv = stat_inv[cv2.CC_STAT_WIDTH]
            h_inv = stat_inv[cv2.CC_STAT_HEIGHT]
            area_inv = stat_inv[cv2.CC_STAT_AREA]
            
            #get distance in pixels between centroids of the two points
            distance = m.sqrt((x_inv-x)**2 + (y_inv-y)**2)
            
            if distance >= DISTANCE_THRESH -10 and distance <= DISTANCE_THRESH +10:
                if area_inv > area:
                    if area > 1000 and area_inv > 15000:
                        #if area_inv >= 6*area:
                        Cx = int(x_inv+w_inv/2)
                        Cy = int(y_inv+h_inv/2)
                        marker_list.append((Cx, Cy))
                        
                        output_img = cv2.drawMarker(marker_img, (Cx,Cy), (0,0,255), cv2.MARKER_CROSS, 5)
                        cv2.putText(output_img, str(len(marker_list)), org=(x,y-15), color=(0,0,255), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, thickness=2)
                        print("area of marker ", len(marker_list), "is ", area, "and inverse is ", area_inv, "with distance ", distance, "\n")
                        found_circle = True
    if found_circle:
        if Cx < .5*IMG_WIDTH and Cy < .5*IMG_HEIGHT:
            quadrant = 2
        if Cx > .5*IMG_WIDTH and Cy < .5*IMG_HEIGHT:
            quadrant = 1
        if Cx > .5*IMG_WIDTH and Cy > .5*IMG_HEIGHT:
            quadrant = 4
        if Cx < .5*IMG_WIDTH and Cy > .5*IMG_HEIGHT:
            quadrant = 3
        cv2.imshow("foundCircle", output_img)
        print("quadrant: ", quadrant, "\n")
        cv2.waitKey(60)
    if not found_circle:
        print("no target found \n")
    
    return quadrant
    
def main():
    
    while True:
        writeByte(find_target())
        
    
if __name__ == "__main__":
    main()
