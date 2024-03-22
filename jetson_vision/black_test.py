import cv2
import numpy as np


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=90,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def empty(a):
    pass

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
cv2.resizeWindow("TrackBars",640,240)
cv2.createTrackbar("Hue Min","TrackBars",0,179,empty)
cv2.createTrackbar("Hue Max","TrackBars",19,179,empty)
cv2.createTrackbar("Sat Min","TrackBars",110,255,empty)
cv2.createTrackbar("Sat Max","TrackBars",240,255,empty)
cv2.createTrackbar("Val Min","TrackBars",153,255,empty)
cv2.createTrackbar("Val Max","TrackBars",255,255,empty)
cv2.createTrackbar("pack1","TrackBars",0,100,empty)
cv2.createTrackbar("pack2","TrackBars",0,100,empty)
cv2.createTrackbar("pack3","TrackBars",0,100,empty)




video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
if video_capture.isOpened():
    try:
        while True:
            ret_val, img = video_capture.read()
            img
            imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
            h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
            s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
            s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
            v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
            v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
            pack1 = cv2.getTrackbarPos("pack1", "TrackBars") /100.0
            pack2 = cv2.getTrackbarPos("pack2", "TrackBars") /100.0
            pack3 = cv2.getTrackbarPos("pack3", "TrackBars") /100.0

            print(h_min,h_max,s_min,s_max,v_min,v_max)

            imgHSV[:,:,0] = imgHSV[:,:,0] * pack1

            imgHSV[:,:,1] = imgHSV[:,:,1] * pack2

            imgHSV[:,:,2] = imgHSV[:,:,2] * pack3

            lower = np.array([h_min,s_min,v_min])
            upper = np.array([h_max,s_max,v_max])
            mask = cv2.inRange(imgHSV,lower,upper)  
            imgResult = cv2.bitwise_and(img,img,mask=mask)


            # cv2.imshow("Original",img)
            # cv2.imshow("HSV",imgHSV)
            # cv2.imshow("Mask", mask)
            # cv2.imshow("Result", imgResult)

            imgStack = stackImages(0.6,([img,imgHSV],[mask,imgResult]))
            cv2.imshow("Stacked Images", imgStack)

            cv2.waitKey(1)
    finally:
            video_capture.release()
            cv2.destroyAllWindows()
else:
    print("Error: Unable to open camera")


#89 131 106 223 0 255   azul
    #89 131 106 223 0 255 amarillo


