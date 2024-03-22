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
cv2.resizeWindow("TrackBars",600,500)
cv2.createTrackbar("Hue Min","TrackBars",0,500,empty)
cv2.createTrackbar("Hue Max","TrackBars",19,500,empty)
cv2.createTrackbar("Sat Min","TrackBars",110,500,empty)
cv2.createTrackbar("Sat Max","TrackBars",240,500,empty)
cv2.createTrackbar("Val Min","TrackBars",153,500,empty)
cv2.createTrackbar("Val Max","TrackBars",255,500,empty)
cv2.createTrackbar("Hue","TrackBars",100,1000,empty)
cv2.createTrackbar("Saturation","TrackBars",100,1000,empty)
cv2.createTrackbar("Value","TrackBars",100,1000,empty)
cv2.createTrackbar("Beta","TrackBars",0,4000,empty)
cv2.createTrackbar("Alfa","TrackBars",0,1000,empty)



video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
if video_capture.isOpened():
    try:
        while True:
            ret_val, img = video_capture.read()
            img = cv2.convertScaleAbs(img, alpha=cv2.getTrackbarPos("Alfa","TrackBars")/100, beta=cv2.getTrackbarPos("Beta","TrackBars")/100)
            imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
            h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
            s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
            s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
            v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
            v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
            pack1 = cv2.getTrackbarPos("Hue", "TrackBars") /100.0
            pack2 = cv2.getTrackbarPos("Saturation", "TrackBars") /100.0
            pack3 = cv2.getTrackbarPos("Value", "TrackBars") /100.0

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



#AMARILLO CABRON 
#HIM = 13
#HMAX = 26
#SMIN = 49
#SMAX = 157
#VALMIN = 82
#VALMAX = 104
#HUE = 50  SOBRE 1000
#SAT = 91 SOBRE 1000
#VALUE = 137 SOBRE 1000
#BETA = 178 SOBRE 1000
#ALPHA = 200 SOBRE 1000


#AZUL CABRON 
#HIM = 47
#HMAX = 68
#SMIN = 46
#SMAX = 150
#VALMIN = 140
#VALMAX = 255
#HUE = 50  SOBRE 1000
#SAT = 91 SOBRE 1000
#VALUE = 103 SOBRE 1000
#BETA = 178 SOBRE 1000
#ALPHA = 200 SOBRE 1000






#ROJO = 
#HIM = 50
#HMAX = 66
#SATMIN = 139
#SATMAX = 255
#VALMIN = 0
#VALMAX = 255
#HUE = 31
#SATURACION = 214
#VALUE = 129
#BETA = 172
#ALFA = 218
    
#VALORES DE PRUEBA ELIMINACION DE RUIDO
#XD H = 50
#XD SAT = 91
#VALUE = 103
#BETA = 137
#ALPHA = 241