import cv2
import numpy as np
import camera_activate

def get_color(img,hmin,hmax,smin,smax,vmin,vmax,hue,sat,val,alpha,beta,erode = 0 , dilate=0):
    img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    imgHSV[:,:,0] = imgHSV[:,:,0] * hue
    imgHSV[:,:,1] = imgHSV[:,:,1] * sat
    imgHSV[:,:,2] = imgHSV[:,:,2] * val
    lower = np.array([hmin,smin,vmin])
    upper = np.array([hmax,smax,vmax])
    imgResult = cv2.inRange(imgHSV,lower,upper)  
    #imgResult = cv2.bitwise_and(img,img,mask=mask)
    #testing filtering noise
    imgResult = cv2.erode(imgResult, None, iterations=erode)
    imgResult = cv2.dilate(imgResult, None, iterations=dilate)

    return imgResult
     

video_capture = cv2.VideoCapture(camera_activate.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

if video_capture.isOpened():
    try:
        while True:
            ret_val, img = video_capture.read()
            img_red = get_color(img,50,66,139,255,0,255,0.31,2.14,1.29,2.18,1.72,2,10)
            img_yellow = get_color(img,13,26,49,157,82,104,0.5,0.91,1.37,1.78,2,1,5)
            img_blue = get_color(img,47,68,46,150,140,255,0.5,0.91,1.03,1.78,2,3,3)

            cv2.imshow("Original",img)
            cv2.imshow("Red",img_red)
            cv2.imshow("Yellow",img_yellow)
            cv2.imshow("Blue",img_blue)


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
            video_capture.release()
            cv2.destroyAllWindows()
else:
    print("Error: Unable to open camera")


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