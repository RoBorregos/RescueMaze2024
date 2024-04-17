import cv2
import camera_activate

def record_video(filename, fps=30, frame_size=(640, 328)):
  """
  Records a video from the camera and saves it to a file.

  Args:
      filename: The name of the output video file (including extension).
      fps: The desired frame rate of the video (default: 25).
      frame_size: The desired size of the video frames (default: (640, 480)).

  Returns:
      None
  """

    # Open the default video capture object (webcam)
  right_cam = camera_activate.gstreamer_pipeline(sensor_id=0,flip_method=0)
  cap = cv2.VideoCapture(right_cam, cv2.CAP_GSTREAMER)

  # Check if camera opened successfully
  if not cap.isOpened():
    print("Error opening camera!")
    return

  # Define the video writer with desired codec, frame rate, and frame size
  fourcc = cv2.VideoWriter_fourcc(*'XVID')  # You can choose other codecs like 'MJPG'
  out = cv2.VideoWriter(filename, fourcc, fps,frame_size )

  while True:
    # Capture frame-by-frame
    try:
      ret, frame = cap.read()
      if not ret:
        print("Error reading frame!")
        break
      
      # Display the resulting frame (optional)
      # cv2.imshow('Recording...', frame)

      # Press 'q' to quit recording
      if cv2.waitKey(1) == ord('q'):
        break
      frame = frame[92:420, 0:640]
      #print(frame.shape)
      # Write the frame to the video writer
      out.write(frame)
    except:
      break
  # Release the capture and video writer objects
  cap.release()
  out.release()
  cv2.destroyAllWindows()
  print(f"Recording saved as {filename}")

# Example usage
filename = "my_recording.avi"
record_video(filename)
