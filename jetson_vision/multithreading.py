import cv2
import numpy as np
import camera_activate
import time as t 
from PIL import Image
import torch
import torch.nn as nn
from torchvision import models, transforms
import serial


################################################################
###############COLOR SEGMENTATION FUNCTIONS#####################

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
    minArea = 1000
    imgResult = areaFilter(minArea, imgResult)

    return imgResult
     
def generate_bbox(img,frame,text=""):
    mask_ = Image.fromarray(img)
    bbox = mask_.getbbox()
    if bbox is not None:
        x1,y1,x2,y2 = bbox
        frame = cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 5)
        if text:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, text, (x1, y1-5), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return True
    else:
        frame = frame
        return False


################################################################
###############MODEL FUNCTIONS###############################

def start_model(): 
    model_ft = models.resnet18(weights='IMAGENET1K_V1')
    num_ftrs = model_ft.fc.in_features
    model_ft.fc = nn.Linear(num_ftrs, len(class_names))
    model_ft = model_ft.to(device)
    model_ft.load_state_dict(torch.load('/home/jetson/RescueMaze2024/jetson_vision/model.pth'))
    model_ft.eval()
    return model_ft


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
        #print(output)
        _, predicted = torch.max(output, 1)
        value = torch.max(output).item()
        if value >= minimum_predict_value:
            return class_names[predicted]
        else:
            return "m"
        #print(output)
    #print(t.time()-time1)

def warmup():
    print("STARTING WARMUP .....")
    actual_time = t.time() + 10
    warm_time = t.time()
    passed_times = 0
    while abs(t.time()-actual_time) > 3:
        actual_time = t.time()
        img = cv2.imread("/home/jetson/RescueMaze2024/jetson_vision/warmup.jpg")
        img = cv2.resize(img,(32,32))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        predict_image(model_ft,img,device,class_names)
        passed_times += 1
    print(f"WARMUP FINISHED {passed_times} times predicted in {t.time()-warm_time}")

################################################################
###############PROCESSING IMAGES FUNCTIONS###############################

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
    alpha = 1.15
    beta=1.4
    img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
    # cv2.imshow("brighter", img)
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

def generate_frame_cut(img):
    mask_ = Image.fromarray(img)
    bbox = mask_.getbbox()
    if bbox is not None :
        x1,y1,x2,y2 = bbox
        frame = img[y1:y2,x1:x2]
    else:
        frame = None

    return frame



def post_processing(img,dilate):
    size_img = cv2.resize(img,(100,100))
    size_img = cv2.dilate(size_img, None, iterations=dilate)
    size_img = cv2.resize(img,(32,32))
    return size_img



################################################################
###############VARIABLES AND MORE###############################

class_names = ['h', 's', 'u']
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(device)
camera_source = camera_activate.gstreamer_pipeline(flip_method=0)
video_capture = cv2.VideoCapture(camera_source, cv2.CAP_GSTREAMER)
#video_capture = cv2.VideoCapture(camera_source)
print("Opening Serial ..")
arduino = serial.Serial('/dev/ttyUSB0', 115200)
print("Serial Opened ..")
print("Loading model ...")
loading_time = t.time()
model_ft = start_model()
print(f"Model loaded in {t.time()-loading_time}")
minimum_predict_value = 1.2
warmup()
fourcc = cv2.VideoWriter_fourcc(*'XVID') 
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480)) 

def main(): 
    print("STARTING VIDEO ....")
    if video_capture.isOpened():
        try:
            # time = t.time()
            letters = ['s','h','u','m']
            repetitions = [0,0,0,0]
            index = 0
            active_camera = 0
            print("VIDEO STARTED")
            while True:
                ret_val, img = video_capture.read()
                if img  is not None and active_camera > 11:
                    actual_state = "m"

                    # PROCESS COLORS
                    img_red = get_color(img,50,66,139,255,0,255,0.31,2.14,1.29,2.18,1.72,2,10)
                    img_yellow = get_color(img,13,26,49,157,82,104,0.5,0.91,1.37,1.78,2,1,5)
                    # img_green = get_color(img,47,68,46,150,140,255,0.5,0.91,1.03,1.78,2,3,3)

                    # #PROCESS LETTERS
                    frame = img.copy()
                    processed_red = generate_bbox(img_red,frame,"red")
                    processed_yellow = generate_bbox(img_yellow,frame,"yellow")
                    # processed_green = generate_bbox(img_green,frame,"yellow")
                    if processed_red:
                        actual_state = "h"
                    if processed_yellow:
                        actual_state = "s"
                    # if processed_green:
                    #     actual_state = "u"
                    

                    binary_img = process_image(img)
                    #cv2.imshow("binary", binary_img)      
                    new_img = rotate_image(binary_img)
                    #cv2.imshow("Rotated", new_img)
                    new_img = generate_frame_cut(new_img)

                    if new_img is not None:
                        new_img = post_processing(new_img,4)
                        new_img = cv2.cvtColor(new_img, cv2.COLOR_GRAY2RGB)
                        actual_letter = predict_image(model_ft,new_img,device,class_names)
                        actual_state = actual_letter
                        # if actual_letter != "m":
                        #     if last_letter == actual_letter:
                        #         letter_count += 1
                        #     else:
                        #         last_letter == actual_letter
                        #         letter_count = 0
                        # else:
                        #     letter_count = 0
                        #     last_letter = "m"
                        #     actual_state = "m"
                            
                        if actual_state != "m":
                           generate_bbox(binary_img,frame,actual_state)

                    #cv2.imshow("Original",frame)
                    out.write(frame)
                    # Update counts
                    for letter in letters:
                        if letter == actual_state:
                            index = letters.index(letter)
                            repetitions[index] += 1
                    print(f"Actual value: {actual_state}  reps: {repetitions[index]}")
                    #for testing
                    # if t.time() > time + 5:
                    #     time = t.time()
                    #     lastMax = 0
                    #     for i in range(4):
                    #         if repetitions[i] > lastMax:
                    #             index = i
                    #             lastMax = repetitions[i]
                    #     actual_state = letters[index]
                    #     print(f"Sending state to esp = {actual_state}")
                    #     for i in range(4):
                    #         print(f"Letter {letters[i]} = {repetitions[i]}")
                    #     repetitions = [0,0,0,0]
                    # If recieving data from arduino
                    if arduino.in_waiting > 0:
                        line = arduino.readline().decode('utf-8').strip()
                        if line == "1":
                            lastMax = 0
                            for i in range(4):
                                if repetitions[i] > lastMax:
                                    index = i
                                    lastMax = repetitions[i]
                            actual_state = letters[index]
                            repetitions = [0,0,0,0]
                            print(f"Sending state to esp = {actual_state}")
                            arduino.write(actual_state.encode('utf-8'))
                        if line == "2":
                            print("Serial reseted")
                            repetitions = [0,0,0,0]
                    
                    #cv2.imshow("Original",img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                active_camera += 1

        finally:
                print("Finishing the program")
                video_capture.release()
                out.release()  
                cv2.destroyAllWindows()
                arduino.close()

    else:
        print("Error: Unajeble to open camera")
        out.release()  
        arduino.close()

if __name__ == '__main__':
    main()