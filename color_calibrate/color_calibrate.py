import cv2
import numpy as np
import camera_activate
import json

def empty(data):
    pass

def download_json():
    with open('calibrated_colors_test.json','r') as file:
        data = json.load(file)
        file.close()
    return data

def write_json(data):
    data = json.dumps(data,indent=4)
    with open('calibrated_colors_test.json','w') as file:
        file.write(data)
        file.close()
def assign_colors(color,data):
    color = data[color]
    color["hmin"] = h_min
    color["hmax"] = h_max
    color["smin"] = s_min
    color["smax"] = s_max
    color["vmin"] = v_min
    color["vmax"] = v_max
    color["hue"] = pack1
    color["sat"] = pack2
    color["val"] = pack3
    color["alpha"]=alpha
    color["beta"]=beta
    color["erode"]=erode
    color["dilate"]=dilate
    return color

def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",600,500)
cv2.createTrackbar("Hue Min","TrackBars",0,255,empty)
cv2.createTrackbar("Hue Max","TrackBars",0,255,empty)
cv2.createTrackbar("Sat Min","TrackBars",0,255,empty)
cv2.createTrackbar("Sat Max","TrackBars",0,255,empty)
cv2.createTrackbar("Val Min","TrackBars",0,255,empty)
cv2.createTrackbar("Val Max","TrackBars",0,255,empty)
cv2.createTrackbar("Hue","TrackBars",100,1000,empty)
cv2.createTrackbar("Saturation","TrackBars",100,1000,empty)
cv2.createTrackbar("Value","TrackBars",100,1000,empty)
cv2.createTrackbar("Beta","TrackBars",100,4000,empty)
cv2.createTrackbar("Alfa","TrackBars",100,1000,empty)
cv2.createTrackbar("erode","TrackBars",0,30,empty)
cv2.createTrackbar("dilate","TrackBars",0,30,empty)





video_capture = cv2.VideoCapture(camera_activate.gstreamer_pipeline(sensor_id=1,flip_method=0), cv2.CAP_GSTREAMER)
if video_capture.isOpened():
    try:
        data = download_json()
        while True:
            ret_val, img = video_capture.read()
            img = img[92:420, 0:640]
            
            
            h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
            h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
            s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
            s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
            v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
            v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
            pack1 = cv2.getTrackbarPos("Hue", "TrackBars") /100.0
            pack2 = cv2.getTrackbarPos("Saturation", "TrackBars") /100.0
            pack3 = cv2.getTrackbarPos("Value", "TrackBars") /100.0
            alpha=cv2.getTrackbarPos("Alfa","TrackBars")/100
            beta=cv2.getTrackbarPos("Beta","TrackBars")/100
            erode=cv2.getTrackbarPos("erode","TrackBars")
            dilate=cv2.getTrackbarPos("dilate","TrackBars")

            img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
            imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            imgHSV[:,:,0] = imgHSV[:,:,0] * pack1
            imgHSV[:,:,1] = imgHSV[:,:,1] * pack2
            imgHSV[:,:,2] = imgHSV[:,:,2] * pack3
            lower = np.array([h_min,s_min,v_min])
            upper = np.array([h_max,s_max,v_max])
            mask = cv2.inRange(imgHSV,lower,upper)  
            # imgResult = cv2.bitwise_and(img,img,mask=mask)
            imgerode = cv2.erode(mask, None, iterations=erode)
            imgdilated = cv2.dilate(imgerode, None, iterations=dilate) 


            imgStack = stackImages(0.6,([img,imgHSV],[mask,imgdilated]))
            cv2.imshow("Stacked Images", imgStack)

            letter = cv2.waitKey(1)
            if letter == ord('q'): break
            if letter == ord('y'):
                data["yellow"] = assign_colors("yellow",data)
                write_json(data)
                print("COLOR YELLOW SAVED")
            if letter == ord('g'):
                data["green"] = assign_colors("green",data)
                write_json(data)
                print("COLOR GREEN SAVED")
            if letter == ord('r'):
                data["red"] = assign_colors("red",data)
                write_json(data)
                print("COLOR RED SAVED")
    finally:
            video_capture.release()
            cv2.destroyAllWindows()
else:
    print("Error: Unable to open camera")


