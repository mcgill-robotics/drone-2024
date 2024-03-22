 import cv2

camera = cv2.VideoCapture(0, cv2.CAP_V4L) #index of camera device which is 0 with one camera

if (camera.isOpened()):
    print("The camera is successfully opened")
else:
   print("Could not open the camera")

frameWidth = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
frameHeight = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
frameRate = int(camera.get(cv2.CAP_PROP_FPS))

# for .avi files use DIVX, XVID
# for .mp4 files use MJPG
fourccCode = cv2.VideoWriter_fourcc(*'DIVX')

videoFileName = 'recordedVideo.avi' #put whtvr video file name is

videoDimension = (frameWidth, frameHeight) # defines dimensions of video

recordedVideo = cv2.VideoWriter(videoFileName, fourccCode, frameRate, videoDimension) # creates a Video Writer object

while True:
    # captures the frame
    success, frame = camera.read()

    if not success:
       print('Not able to read the frame. End')
       break

    cv2.imshow('Camera video', frame) # display camera image

    recordedVideo.write(frame) # saving frames to the file frame

    if cv2.waitKey(1) == ord('c'): # if press c the video capturing will end
       break

recordedVideo.release()
camera.release()
cv2.destroyAllWindows()