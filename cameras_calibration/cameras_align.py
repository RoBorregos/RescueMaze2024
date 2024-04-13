import camera_activate
import cv2
import numpy as np

def empty(a):
    pass

cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",50,200)
cv2.createTrackbar("Frame 1","TrackBars",444,480,empty)
cv2.createTrackbar("Frame 2","TrackBars",393,480,empty)
cv2.createTrackbar("Frame 1 top","TrackBars",0,480,empty)
cv2.createTrackbar("Frame 2 top","TrackBars",92,480,empty)

def show_camera():
    window_title = "CSI Camera"

    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    video_capture = cv2.VideoCapture(camera_activate.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    video_capture_2 = cv2.VideoCapture(camera_activate.gstreamer_pipeline(sensor_id=0,flip_method=0), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            while True:
                ret_val, frame = video_capture.read()
                ret_val_2, frame_2 = video_capture_2.read()
                #replace pixels from above with white
                frame1 = cv2.getTrackbarPos("Frame 1","TrackBars")
                frame2 = cv2.getTrackbarPos("Frame 2", "TrackBars")
                frame1top = cv2.getTrackbarPos("Frame 1 top","TrackBars")
                frame2top = cv2.getTrackbarPos("Frame 2 top", "TrackBars")
                # print(frame_1)
                # print(frame_2)
                # print(frame.shape)
                # frame = frame[frame1:480, 0:640]
                # frame_2 = frame_2[frame2:480, 0:640]
                # frame = frame[0:frame1top, 0:640]
                # frame_2 = frame_2[0:frame2top, 0:640]
                frame[frame1:480, 0:640] = [255, 255, 255]
                frame_2[frame2:480, 0:640] = [255, 255, 255]
                frame[0:frame1top, 0:640] = [255, 255, 255]
                frame_2[0:frame2top, 0:640] = [255, 255, 255]
                #resize frame
                # frame = cv2.resize(frame, (640, 480))
                # frame_2 = cv2.resize(frame_2, (640, 480))
                combined_img = cv2.hconcat([frame, frame_2])
                cv2.imshow("Prueba",combined_img)
                #show frame and frame_2 in one window

                keyCode = cv2.waitKey(10) & 0xFF
                # Stop the program on the ESC key or 'q'
                if keyCode == 27 or keyCode == ord('q'):
                    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    show_camera()
