import cv2
import numpy as np
import camera_activate
import time as t 
from PIL import Image
import torch
import torch.nn as nn
from torchvision import models, transforms
import serial

class_names = ['h', 's', 'u']
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(device)

def get_color(img,hmin,hmax,smin,smax,vmin,vmax,hue,sat,val,alpha,beta,erode = 0,dilate=0):
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
     
def generate_bb(img,frame,text="",showbbox=False):
    mask_ = Image.fromarray(img)
    bbox = mask_.getbbox()
    if bbox is not None and showbbox:
        x1,y1,x2,y2 = bbox
        frame = cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 5)
        if text:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, text, (x1, y1-5), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return True 
    elif bbox is not None:
        return True
    else:
        frame = frame
        return False
    





def start_model():
    
    model_ft = models.resnet18(weights='IMAGENET1K_V1')
    num_ftrs = model_ft.fc.in_features
    model_ft.fc = nn.Linear(num_ftrs, len(class_names))
    model_ft = model_ft.to(device)
    model_ft.load_state_dict(torch.load('model.pth'))
    model_ft.eval()
    return model_ft

def areaFilter(minArea, inputImage):
    # Perform an area filter on the binary blobs:
    componentsNumber, labeledImage, componentStats, componentCentroids = \
        cv2.connectedComponentsWithStats(inputImage, connectivity=4)
    # Get the indices/labels of the remaining components based on the area stat
    # (skip the background component at index 0)
    remainingComponentLabels = [i for i in range(1, componentsNumber) if componentStats[i][4] >= minArea]
    # Filter the labeled pixels based on the remaining labels,
    # assign pixel intensity to 255 (uint8) for the remaining pixels
    filteredImage = np.where(np.isin(labeledImage, remainingComponentLabels) == True, 255, 0).astype('uint8')

    return filteredImage


def rotate_image(binary_img):
    coords = np.column_stack(np.where(binary_img > 0))
    angle = cv2.minAreaRect(coords)[-1]
    if angle < -45:
        angle = -(90 + angle)
    else:
        angle = -angle
    (h, w) = binary_img.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(binary_img, M, (w, h), flags=cv2.INTER_CUBIC)
    return rotated

def process_image(img):
    imgFloat = img.astype(float) / 255.0
    kChannel = 1 - np.max(imgFloat, axis=2)
    kChannel = (255*kChannel).astype(np.uint8)
    #cv2.imshow('kChannel', kChannel)
    binaryThresh = 170
    _, binaryImage = cv2.threshold(kChannel, binaryThresh, 255, cv2.THRESH_BINARY)
    #cv2.imshow("binary", binaryImage)
    kernelSize = 3
    opIterations = 2
    morphKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernelSize, kernelSize))
    binaryImage = cv2.morphologyEx(binaryImage, cv2.MORPH_CLOSE, morphKernel, None, None, opIterations, cv2.BORDER_REFLECT101)
    #cv2.imshow("binary2", binaryImage)
    #dudoso
    minArea = 1000
    filteredImage = areaFilter(minArea, binaryImage)
    #cv2.imshow("filtered", filteredImage)
    return filteredImage


    
camera_source = camera_activate.gstreamer_pipeline(flip_method=0)


video_capture = cv2.VideoCapture(camera_source, cv2.CAP_GSTREAMER)
#video_capture = cv2.VideoCapture(camera_source)

def generate_bbox(img):
    mask_ = Image.fromarray(img)
    bbox = mask_.getbbox()
    if bbox is not None :
        x1,y1,x2,y2 = bbox
        frame = img[y1:y2,x1:x2]
    else:
        frame =""
    
    return frame

def predict_image(model, image_path,device,class_names):
    transform = transforms.Compose([
        transforms.ToTensor(),
    ])
    image = Image.fromarray(image_path)
    image = transform(image).unsqueeze(0)
    image = image.to(device)
    #print("started")
    time1 = t.time()
    with torch.no_grad():
        output = model(image)
        _, predicted = torch.max(output, 1)
    #print(t.time()-time1)
    return class_names[predicted]

def post_processing(img,dilate):
    size_img = cv2.resize(img,(100,100))
    size_img = cv2.dilate(size_img, None, iterations=dilate)
    size_img = cv2.resize(img,(32,32))
    return size_img
    

model_ft = start_model()
count_for_process = 10
count_for_model = 4
count_process = 0
process_mid = 4
last_state = ""
arduino = serial.Serial('/dev/ttyUSB0', 115200)
if video_capture.isOpened():
    try:
        count = 0
        count_one = 0
        while True:
            ret_val, img = video_capture.read()


            # img_red = get_color(img,50,66,139,255,0,255,0.31,2.14,1.29,2.18,1.72,2,10)
            # img_yellow = get_color(img,6,39,0,255,0,255,0.58,1.7,1.03,2.04,2,5,5)
            # img_blue = get_color(img,47,68,46,150,140,255,0.5,0.91,1.03,1.78,2,3,3)

            img_red = get_color(img,50,66,139,255,0,255,0.31,2.14,1.29,2.18,1.72,2,10)
            img_yellow = get_color(img,13,26,49,157,82,104,0.5,0.91,1.37,1.78,2,1,5)
            img_blue = get_color(img,47,68,46,150,140,255,0.5,0.91,1.03,1.78,2,3,3)


            #BOUNDING BOX GENERATION
            frame = img.copy()
            actual_state = "h" if generate_bb(img_red,frame,"Red",True) else None
            actual_state = "s" if generate_bb(img_yellow,frame,"Yellow",True) else None
            actual_state = "u" if generate_bb(img_blue,frame,"Blue",True) else None


            binary_img = process_image(img)
            #cv2.imshow("binary", binary_img)      
            new_img = rotate_image(binary_img)
            #cv2.imshow("Rotated", new_img)
            new_img = generate_bbox(new_img)
            
            if new_img != "" and count >= count_for_process:

                new_img = post_processing(new_img,4)
                new_img = cv2.cvtColor(new_img, cv2.COLOR_GRAY2RGB)
                
                if count_one >=count_for_model:
                    
                    # print(predict_image(model_ft,new_img,device,class_names))
                    
                
                    actual_state = predict_image(model_ft,new_img,device,class_names)
                    #actual_state = "u"
                    if actual_state != last_state:

                        count_process  = 0
                        last_state = actual_state

                    elif actual_state == last_state and count_process < process_mid:
                        count_process +=1
                    else:
                        print("letter is: "+actual_state)
                        count_process = 0
                        last_state = ""

               # cv2.imshow("Final", new_img)
                count_one +=1
            else:
                count_one = 0
                actual_state = "m"
            count +=1
            

            if arduino.in_waiting > 0:
                line = arduino.readline().decode('utf-8').strip()  # Read a line from the serial port
                print("Triggered")  # Print the received line
        
        # Example of sending data from Python to Arduino
                if line == "1":
                    print(f"SENDING STATE = {actual_state}")
                    arduino.write(actual_state.encode('utf-8'))  # Send a string to Arduino
            
           # cv2.imshow("Original",img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
            video_capture.release()
            cv2.destroyAllWindows()
            arduino.close()
else:
    print("Error: Unable to open camera")
    arduino.close()
