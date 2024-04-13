import cv2
import numpy as np
import camera_activate
import time as t 
from PIL import Image
import torch
import torch.nn as nn
from torchvision import models, transforms
import serial
import calibrated_colors


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
    return imgResult
     
def generate_bbox(img,frame,text=""):
    mask_ = Image.fromarray(img)
    bbox = mask_.getbbox()
    if bbox is not None:
        x1,y1,x2,y2 = bbox
        frame = cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 5)
        if text:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, text, (x1, y2+5), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return True
    else:
        frame = frame
        return False


################################################################
##################MODEL FUNCTIONS###############################

def start_model(): 
    model_ft = models.resnet18(weights='IMAGENET1K_V1')
    num_ftrs = model_ft.fc.in_features
    model_ft.fc = nn.Linear(num_ftrs, len(class_names))
    model_ft = model_ft.to(device)
    model_ft.load_state_dict(torch.load('model.pth'))
    model_ft.eval()
    return model_ft

def predict_image(model, image_path,device,class_names):
    global value_predict
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
            value_predict = value
            return class_names[predicted]
        else:
            return "m"

def warmup():
    print("STARTING WARMUP .....")
    actual_time = t.time() + 10
    warm_time = t.time()
    passed_times = 0
    while abs(t.time()-actual_time) > 3:
        actual_time = t.time()
        img = cv2.imread("warmup.jpg")
        img = cv2.resize(img,(32,32))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        predict_image(model_ft,img,device,class_names)
        passed_times += 1
    print(f"WARMUP FINISHED {passed_times} times predicted in {t.time()-warm_time}")

#########################################################################
###############PROCESSING IMAGES FUNCTIONS###############################

def areaFilter(minArea, inputImage):
    componentsNumber, labeledImage, componentStats, componentCentroids = \
        cv2.connectedComponentsWithStats(inputImage, connectivity=4)
    remainingComponentLabels = [i for i in range(1, componentsNumber) if componentStats[i][4] >= minArea]
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
    global binary_process
    alpha = 1
    beta=1
    img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
    #cv2.imshow("brighter", img)
    imgFloat = img.astype(float) / 255.0
    kChannel = 1 - np.max(imgFloat, axis=2)
    kChannel = (255*kChannel).astype(np.uint8)
    #cv2.imshow('kChannel', kChannel)
    binaryThresh = 170
    _, binaryImage = cv2.threshold(kChannel, binaryThresh, 255, cv2.THRESH_BINARY)
    binary_process = binaryImage
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
###############EXTERNAL FUNCTIONS###############################

def start_serial():
    print("Opening Serial ..")
    while True:
        try:
            arduino = serial.Serial('/dev/ttyUSB0', 115200)
            break
        except:
            print("Serial not found .. waiting 2 seconds")
            t.sleep(2)
    print("Serial Opened ..")
    return arduino


################################################################
###############MAIN STARTED FUNCTON###############################
def setup():
    #Main variables
    global right_video
    global left_video
    global class_names
    global device
    global arduino
    global model_ft
    global minimum_predict_value
    #Debug variables
    global out
    global value_predict
    global binary_process
    global debug

    ##################VARIABLES ZONE#################
    class_names = ['h', 's', 'u']
    minimum_predict_value = 0
    #GLOBAL DEBUG VARIABLES
    debug = {
        'arduino':False,
        'model':False,
        'record':False,
        'generate_BBOX':False,
    }
    value_predict = 0
    binary_process = None
    #Start record
    if debug['record']:
        fourcc = cv2.VideoWriter_fourcc(*'XVID') 
        out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480)) 
    ##################DEVICES SETUP##################
    #DEVICE TORCH USAGE
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(device)
    #CAMERA ASSIGNMENT
    right_cam = camera_activate.gstreamer_pipeline(flip_method=0)
    right_video = cv2.VideoCapture(right_cam, cv2.CAP_GSTREAMER)
    left_cam= camera_activate.gstreamer_pipeline(sensor_id=0,flip_method=0)
    left_video = cv2.VideoCapture(left_cam, cv2.CAP_GSTREAMER)
    #MODEL SETUP AND WARMUP
    if debug['model']:
        print("Loading model ...")
        loading_time = t.time()
        model_ft = start_model()
        print(f"Model loaded in {t.time()-loading_time}")
        warmup()
    #SERIAL SETUP
    if debug["arduino"] : arduino = start_serial()


def main(): 
    ############SETUP#############
    print("STARTING SETUP....")
    setup_time = t.time()
    setup()
    print(f"Setup finished in {t.time()-setup_time}")
    print("STARTING VIDEO ....")
    ############LOOP###############
    if right_video.isOpened() and left_video.isOpened():
        try:
            actual_state = "m"
            active_camera  = 0
            frames = 0
            print("VIDEO STARTED")
            while True:
                ret_val_r, img_r = right_video.read()
                ret_val_l, img_l = left_video.read()

                if img_r  is not None and active_camera > 11 and img_l is not None:
                    # img_r[444:480, 0:640] = [255, 255, 255]
                    # img_l[393:480, 0:640] = [255, 255, 255]
                    # img_l[0:92, 0:640] = [255, 255, 255]
                    combined_img = cv2.hconcat([img_r, img_l])
                    cv2.imshow("Prueba",combined_img)
                    # actual_state = "m"
                
                    # PROCESS COLORS
                    img_red = get_color(img_r,50,66,139,255,0,255,0.31,2.14,1.29,2.18,1.72,2,10)
                    img_yellow = get_color(img_r,13,26,49,157,82,104,0.5,0.91,1.37,1.78,2,1,5)
                    #img_green = get_color(img,47,68,46,150,140,255,0.5,0.91,1.03,1.78,2,3,3)

                    
                    frame = img_r.copy()
                    processed_red = generate_bbox(img_red,frame,"red")
                    processed_yellow = generate_bbox(img_yellow,frame,"yellow")
                    # processed_green = generate_bbox(img_green,frame,"yellow")

                    if processed_red:
                        actual_state = "h"
                    if processed_yellow:
                        actual_state = "s"
                    # if processed_green:
                    #     actual_state = "u"

                    # #PROCESS LETTERS
                    binary_img = process_image(img_l)
                    #cv2.imshow("binary", binary_img)      
                    new_img = rotate_image(binary_img)
                    #cv2.imshow("Rotated", new_img)
                    new_img = generate_frame_cut(new_img)

                    if new_img is not None:
                        new_img = post_processing(new_img,4)
                        new_img = cv2.cvtColor(new_img, cv2.COLOR_GRAY2RGB)
                        if debug['model']:
                            actual_letter = predict_image(model_ft,new_img,device,class_names)
                            actual_state = actual_letter
                        #DEBUG IN IMAGE
                        if actual_state != "m" and debug['generate_BBOX'] and debug['model']:
                           generate_bbox(binary_img,frame,f"{actual_state} {value_predict}")

                    print(f"Actual value: {actual_state}  frames: {frames}")
                    frames += 1
                    #cv2.imshow("Original",img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    active_camera += 1

        finally:
                print("Finishing the program")
                right_video.release()
                left_video.release()
                if debug['record']: out.release()  
                cv2.destroyAllWindows()
                if debug["arduino"] : arduino.close()

    else:
        print("Error: Unable to open camera")
        if debug['record']: out.release()  
        if debug["arduino"] : arduino.close()

if __name__ == '__main__':
    main()