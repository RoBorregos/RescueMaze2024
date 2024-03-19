import cv2

# Create a VideoCapture object to access the camera
cap = cv2.VideoCapture(0)  # 0 corresponds to the default camera, change it if you have multiple cameras

# Check if the camera was opened successfully
if not cap.isOpened():
    print("Error: Unable to access the camera.")
    exit()

# Capture a frame from the camera
ret, frame = cap.read()

# Check if the frame was captured successfully
if not ret:
    print("Error: Unable to capture frame.")
    cap.release()  # Release the camera device
    exit()

# Display the captured frame (optional)
cv2.imshow("Frame", frame)
cv2.waitKey(0)  # Wait for a key press to exit

# Save the captured frame as an image
cv2.imwrite("../vision/pic.jpg", frame)

# Release the camera device
cap.release()
cv2.destroyAllWindows()  # Close all OpenCV windows
