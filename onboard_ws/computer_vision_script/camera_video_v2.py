import cv2
import time

#index of camera device which is 0 with one camera & DOESNT WORK WITHOUT SECOND PART
camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

if (camera.isOpened()):
   print("The camera is successfully opened")
else:
   print("Could not open the camera")
   exit()

width = int(640 * 3.6)
height = int(480 * 3.6)
fr = 5
# ok = camera.set(cv2.CAP_PROP_MODE, cv2.CAP_MODE_GRAY)
# if (not ok):
#    print("Could not change camera mode")
#    exit()
ok = camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
if (not ok):
   print("Could not change image width")
   exit()
ok = camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
if (not ok):
   print("Could not change image height")
   exit()
#
ok = camera.set(cv2.CAP_PROP_FPS, fr)
if (not ok):
   print("Could not change camera frame rate")
   exit()

time.sleep(2)
frameWidth = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
frameHeight = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
frameRate = int(camera.get(cv2.CAP_PROP_FPS))
print(f"w, h, fr: {frameWidth}, {frameHeight}, {frameRate}")

# for .avi files use DIVX, XVID
# for .mp4 files use MJPG
fourccCode = cv2.VideoWriter_fourcc(*'DIVX')

videoFileName = 'recordedVideo.avi'  #put whtvr video file name is

videoDimension = (frameWidth, frameHeight)  # defines dimensions of video

recordedVideo = cv2.VideoWriter(
   videoFileName, fourccCode, frameRate,
   videoDimension)  # creates a Video Writer object

while True:
   # captures the frame
   success, frame = camera.read()

   if not success:
      print('Not able to read the frame. End')
      break

   cv2.imshow('Camera video', frame)  # display camera image

   recordedVideo.write(frame)  # saving frames to the file frame

   if cv2.waitKey(1) == ord('c'):  # if press c the video capturing will end
      break

recordedVideo.release()
camera.release()
cv2.destroyAllWindows()
