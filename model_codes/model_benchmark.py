
import cv2
import numpy as np
import time as t 
from PIL import Image
import torch
import torch.nn as nn
from torchvision import models, transforms






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
def search_letter(img,frame,actual_state):
    global preprocessing_frame
     # #PROCESS LETTERS
    binary_img = process_image(img)
    #cv2.imshow("binary", binary_img)


    # # find contours
    result = img.copy()
    contours= cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #print(contours)
    contours = contours[0] if len(contours) == 2 else contours[1]
    # print(len(contours))
    last_value_process = minimum_predict_value
    x1b,y1b,x2b,y2b = -12,-12,-12,-12
    for cntr in contours:
        x,y,w,h = cv2.boundingRect(cntr)
        cutted_side = binary_img[y:y+h,x:x+w]
        x1 = 0
        x2 = w
        y1 = 0
        y2 = h
        y1,y2,x1,x2 = [40]*4
        border = cv2.copyMakeBorder(cutted_side, y1, y2, x1, x2, cv2.BORDER_CONSTANT, value=[0, 0, 0])
        rotated_img = rotate_image(border)
        cutted_img = generate_frame_cut(rotated_img)
       
        if cutted_img is not None:
            post_img = post_processing(cutted_img,4)
            new_img = cv2.cvtColor(post_img,cv2.COLOR_GRAY2RGB)
            
            actual_letter = predict_image(model_ft,new_img,device,class_names)
            if value_predict > last_value_process:
                x1b = x
                x2b = x+w
                y1b = y
                y2b = y+h
                last_value_process = value_predict
                actual_state = actual_letter
                    
            #DEBUG IN IMAGE
            if actual_state != "m":
                if x1b & x2b & y1b & y2b != -12:
                    frame = cv2.rectangle(frame, (x1b,y1b), (x2b,y2b), (0,255,0), 5)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    if y1b-5 <= 0:
                        cv2.putText(frame,f"{actual_state} {value_predict}" , (x1b, y2b+15), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    else:
                        cv2.putText(frame,f"{actual_state} {value_predict}" , (x1b, y1b-5), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    
    return actual_state

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
    alpha = 1.01
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
###############MAIN STARTED FUNCTON###############################
def setup():
    #Main variables
    global video_source
    global class_names
    global device
    global model_ft
    global minimum_predict_value

    #Debug variables
    global out
    global value_predict

    ##################VARIABLES ZONE#################
    class_names = ['h', 's', 'u']
    minimum_predict_value = 1.3
    #GLOBAL DEBUG VARIABLES
    value_predict = 0
    #CAMERA ASSIGNMENT
    video_source= cv2.VideoCapture("my_recording.avi")
    #Start record
   
    fourcc = cv2.VideoWriter_fourcc(*'XVID') 
    out = cv2.VideoWriter('benchmark.avi', fourcc, 60.0, (640, 328)) 
    ##################DEVICES SETUP##################
    #DEVICE TORCH USAGE
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(device)
    #MODEL SETUP AND WARMUP
    print("Loading model ...")
    loading_time = t.time()
    model_ft = start_model()
    print(f"Model loaded in {t.time()-loading_time}")
    warmup()


def main(): 
    ############SETUP#############
    print("STARTING SETUP....")
    setup_time = t.time()
    setup()
    print(f"Setup finished in {t.time()-setup_time}")
    print("STARTING VIDEO ....")
    ############LOOP###############
    if video_source.isOpened():
        try:
            active_camera = 0
          
            print("VIDEO STARTED")
            ret_val =True
            while ret_val:
                
                ret_val, img = video_source.read()
                time_diference  = t.time()
                if img is not None:
                    frame = img.copy()
                    detection = search_letter(img,frame,"m")
                    out.write(frame)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    
                    
                else:
                    print("image is none")
                    active_camera += 1

        finally:
                print(f"Finished video saved")
                video_source.release()
                out.release()  
                cv2.destroyAllWindows()

    else:   
        out.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
