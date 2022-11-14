#!/usr/bin/env python3

# Import standard libraries
import cv2
import sdc


def video_cap(result, cap):
  ret, frame = cap.read()
  sdc.pipeline(frame)
  result.write(frame)
  #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  # We need to set resolutions.
  # so, convert them from float to integer.
  

  #cv2.imshow("video", frame)
  if cv2.waitKey(1) & 0xFF == ord('q'):
    cap.release()
    result.release()
    cv2.destroyAllWindows()


cap = cv2.VideoCapture(0) # check this 
if (cap.isOpened() == False): 
    print("Error reading video file")
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
print(f"height: {frame_height}, width: {frame_width}")

size = (frame_width, frame_height)
result = cv2.VideoWriter('filename.mp4', 
                      cv2.VideoWriter_fourcc(*'MP4V'),
                      30, size)
while True:     
  video_cap(result, cap)


