import cv2
import numpy as np
import camera_activate
import time as t 
from PIL import Image


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
    rotated = cv2.warpAffine(binary_img, M, (w, h), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
    return rotated

def process_image(img):
    imgFloat = img.astype(np.float) / 255.0
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

def generate_bbox(img,frame,text="",showbbox=False):
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


if video_capture.isOpened():
    try:
        while True:
            ret_val, img = video_capture.read()
            binary_img = process_image(img)
            cv2.imshow("binary", binary_img)      
            rotated = rotate_image(binary_img)
            cv2.imshow("Rotated", rotated)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
            video_capture.release()
            cv2.destroyAllWindows()
else:
    print("Error: Unable to open camera")


#89 131 106 223 0 255   azul
    #89 131 106 223 0 255 amarillo


