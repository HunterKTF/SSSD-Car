#!/usr/bin/env python3

# Import standard libraries
import cv2
import sdc
import driving_feature as driving


def video_cap(cap, result, result2prev_x1_r, prev_y1_r, prev_x2_r, prev_y2_r, prev_x1_l, prev_y1_l, prev_x2_l, prev_y2_l, prev_x1_hough, prev_y1_hough, prev_x2_hough, prev_y2_hough, car):
  car = driving.Steering()
  car.init_sim_params()
  ret, frame = cap.read()
  sdc.pipeline(frame, result, result2, prev_x1_r, prev_y1_r, prev_x2_r, prev_y2_r, prev_x1_l, prev_y1_l, prev_x2_l, prev_y2_l, prev_x1_hough, prev_y1_hough, prev_x2_hough, prev_y2_hough, car)
  #result.write(frame)
  #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  # We need to set resolutions.
  # so, convert them from float to integer.


  #cv2.imshow("video", frame)
  if cv2.waitKey(1) & 0xFF == ord('q'):
    cap.release()
    result.release()
    result2.release()
    cv2.destroyAllWindows()


"""cap = cv2.VideoCapture(0) # check this
if (cap.isOpened() == False):
    print("Error reading video file")
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
print(f"height: {frame_height}, width: {frame_width}")

size = (frame_width, frame_height)
result = cv2.VideoWriter('filename.avi',
                      cv2.VideoWriter_fourcc(*'MJPG'),
                      20, size)
while True:
  video_cap(cap, result)"""