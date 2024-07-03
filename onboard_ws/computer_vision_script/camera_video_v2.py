import cv2
import time

camera = cv2.VideoCapture(
   "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1920,height=1080, framerate=10/1 ! nvvidconv ! appsink",
   cv2.CAP_GSTREAMER)

if (camera.isOpened()):
   print("The camera is successfully opened")
else:
   print("Could not open the camera")
   exit()

time.sleep(2)
frameWidth = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
frameHeight = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
frameRate = int(camera.get(cv2.CAP_PROP_FPS))
print(f"w, h, fr: {frameWidth}, {frameHeight}, {frameRate}")

while True:
   # captures the frame
   success, frame = camera.read()

   if not success:
      print('Not able to read the frame. End')
      break

   frame = cv2.resize(frame, (1280, 720))
   cv2.imshow('Camera video', frame)  # display camera image

   # recordedVideo.write(frame)  # saving frames to the file frame

   if cv2.waitKey(1) == ord('c'):  # if press c the video capturing will end
      break

# recordedVideo.release()
camera.release()
cv2.destroyAllWindows()
