import cv2
import numpy as np
import camera_activate
import time as t 
from PIL import Image
import torch
import torch.nn as nn
from torchvision import models, transforms
import serial

def start_model(): 
    model_ft = models.resnet18(weights='IMAGENET1K_V1')
    num_ftrs = model_ft.fc.in_features
    model_ft.fc = nn.Linear(num_ftrs, len(class_names))
    model_ft = model_ft.to(device)
    model_ft.load_state_dict(torch.load('/home/jetson/RescueMaze2024/jetson_vision/model.pth'))
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

def generate_bbox(img):
    mask_ = Image.fromarray(img)
    bbox = mask_.getbbox()
    if bbox is not None :
        x1,y1,x2,y2 = bbox
        frame = img[y1:y2,x1:x2]
    else:
        frame = None

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
        #print(output)
        _, predicted = torch.max(output, 1)
        value = torch.max(output).item()
        if value >= minimum_predict_value:
            return class_names[predicted]
        else:
            return "m"
        #print(output)
    #print(t.time()-time1)

def post_processing(img,dilate):
    size_img = cv2.resize(img,(100,100))
    size_img = cv2.dilate(size_img, None, iterations=dilate)
    size_img = cv2.resize(img,(32,32))
    return size_img

class_names = ['h', 's', 'u']
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(device)
camera_source = camera_activate.gstreamer_pipeline(flip_method=0)
video_capture = cv2.VideoCapture(camera_source, cv2.CAP_GSTREAMER)
#video_capture = cv2.VideoCapture(camera_source)
arduino = serial.Serial('/dev/ttyUSB0', 115200)

model_ft = start_model()
minimum_predict_value = 1.0
count_for_process = 10  #Minimum value of consequtive frames with bbox
count_for_model = 4     #Minimum value of M
process_mid = 4


def main(): 
    if video_capture.isOpened():
        try:
            count = 0
            count_one = 0
            last_state = ""
            count_process = 0
            req_count = 0
            while True:
                ret_val, img = video_capture.read()
                binary_img = process_image(img)
                #cv2.imshow("binary", binary_img)      
                new_img = rotate_image(binary_img)
                #cv2.imshow("Rotated", new_img)
                new_img = generate_bbox(new_img)

                

                if new_img is not None:
                    new_img = post_processing(new_img,4)
                    new_img = cv2.cvtColor(new_img, cv2.COLOR_GRAY2RGB)
                    actual_state = predict_image(model_ft,new_img,device,class_names)


                # if new_img != "" and count >= count_for_process:
                #     new_img = post_processing(new_img,4)
                #     new_img = cv2.cvtColor(new_img, cv2.COLOR_GRAY2RGB)
                    
                #     if count_one >=count_for_model:
                #         # print(predict_image(model_ft,new_img,device,class_names))
                #         actual_state = predict_image(model_ft,new_img,device,class_names)
                #         #actual_state = "u"

                #         if actual_state != last_state:
                #             count_process  = 0
                #             last_state = actual_state

                #         elif actual_state == last_state and count_process < process_mid:
                #             count_process +=1

                #         else:
                #             count_process = 0
                #             last_state = ""

                    #cv2.imshow("Final", new_img)
                #     count_one +=1

                # else:
                #     count_one = 0
                #     actual_state = "m"
            
                # count +=1



                #cv2.imshow("Original",img)
                print(f"Actual value: {actual_state}  req: {req_count}")
                req_count += 1
                if arduino.in_waiting > 0:
                    line = arduino.readline().decode('utf-8').strip()

                    if line == "1":
                        print(f"Sending state to esp = {actual_state}")
                        arduino.write(actual_state.encode('utf-8'))
                
                cv2.imshow("Original",img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
                print("Finishing the program")
                video_capture.release()
                cv2.destroyAllWindows()
                # arduino.close()

    else:
        print("Error: Unable to open camera")
        # arduino.close()

if __name__ == '__main__':
    main()