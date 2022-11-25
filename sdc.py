"""
sdc.py

Description: Leer una m¿imagen y detecta lineas 

Author: Yosthin Galindo
Contact: yosthin.galindo@udem.edu
First created: Monday 27 october, 2022
Usage:

"""

# Import required libraries
import numpy as np
import cv2
import argparse
import os
import time

# Import user-defined libraries
import sdc_library 
import driving_feature as driving

def pipeline(img_name, result, result2, prev_x1_r, prev_y1_r, prev_x2_r,
             prev_y2_r, prev_x1_l, prev_y1_l, prev_x2_l,
             prev_y2_l, prev_x1_hough, prev_y1_hough,
             prev_x2_hough, prev_y2_hough, car):
    
    # 1.- Read image
    img_colour = sdc_library.input_image(img_name, result)

    #2.- Convert from BGR to RGB then from RGB to greyscale
    grey = sdc_library.greyscale_img(img_colour)

    h, w, c = img_colour.shape
    points = np.float32([(126, 115), (514, 115), (0, 300), (640, 300)])
    #points = sdc_library.valTrackbars(wT=640, hT=480)
    #print(points)
    imgWarp = sdc_library.warpImg(grey, points, w, h)
    #imgCopy = img_colour.copy()
    #imgWarpPoints = sdc_library.drawPoints(imgCopy, points)

    # 3.- Apply Gaussian smoothing
    kernel_size = (11, 11)
    blur_grey = sdc_library.smoothed_img(imgWarp, kernel_size)
    
    # 4-. Apply Canny edge detector
    low_threshold = 70
    high_threshold = 100
    edges = sdc_library.canny_img(blur_grey, low_threshold, high_threshold)
    
    # 5.- Get a region of interest using the just created polygon
    # Define a Region-of-Interest. Change the below vertices according
    # to input image resolution
    p1, p2, p3, p4 = (3, 438), (25, 100), (600, 100), (630, 438)
    #p1, p2, p3, p4, p5, p6, p7, p8 = (3, 438), (3, 296), (600, 296), (600, 438), (550, 438), (450, 320), (50, 320), (10, 438)
    # create a vertices array that will be used for the roi
    vertices = np.array([[p1, p2, p3, p4]], dtype=np.int32)
    roi_image = sdc_library.region_of_interest(edges, vertices)
    
    # 6.- Apply Hoguh transform for lane lines detection
    rho = 2                             # distance resolution in pixels of the Hough grid
    theta = np.pi/180                   # angular resolution in radians of the Hough grid
    threshold = 100                      # minimum number of votes (intersections in Hough grid)
    min_line_len = 10                    # minimum number of pixels making up a line
    max_line_gap = 30                   # maximum gap in pixels between connectable line segments
    hough_lines, prev_x1_hough, prev_y1_hough, prev_x2_hough, prev_y2_hough = sdc_library.hough(img_colour, roi_image, rho, theta, threshold, min_line_len, max_line_gap, prev_x1_hough, prev_y1_hough, prev_x2_hough, prev_y2_hough)

    # 7-. Get the inlier left and right Hough lines
    left_line_x, left_line_y, right_line_x, right_line_y = sdc_library.left_and_right_lines(hough_lines, img_colour)
    
    # 8-. Draw a single line for the left and right lane lines
    img_lane_lines, prev_x1_r, prev_y1_r, prev_x2_r, prev_y2_r, prev_x1_l, prev_y1_l, prev_x2_l, prev_y2_l = sdc_library.lane_lines(left_line_x, left_line_y, right_line_x, right_line_y, roi_image, result2, prev_x1_r, prev_y1_r, prev_x2_r, prev_y2_r, prev_x1_l, prev_y1_l, prev_x2_l, prev_y2_l) 
    basePoint = sdc_library.getHistogram(img_lane_lines, display=True)  
    
    return basePoint

"""   if __name__ == "__main__":
    
    video = "video3.avi"
    cap = cv2.VideoCapture(video)
    size = (640,480)
    result = cv2.VideoWriter('lane_lines.mp4', 
                        cv2.VideoWriter_fourcc(*'MP4V'),
                        20, size)
    while(cap.isOpened()):
        pipeline(cap, result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # When everything done, release the video capture object
    cap.release()
    result.release()
    # Closes all the frames
    cv2.destroyAllWindows()
    
 
    # Ask the user to enter the path to input images
    parser = argparse.ArgumentParser()
    parser.add_argument("--path_to_images", help="Path to input images")
    args = parser.parse_args()

    # Get the list of image files and sort it alphabetically
    list_with_name_of_images = sorted(os.listdir(args.path_to_images))
    # Loop through each input image
    for im in list_with_name_of_images:
        # Build path and image name
        path_and_im= args.path_to_images+im
        # Get the start time
        start_time = time.process_time()
        # Run the workflow to each input image
        pipeline(path_and_im)
        # Print the name of image being processed and compute FPS
        print(f"Processing image:{path_and_im}",
            f"\tCPU execution time:{1/(time.process_time()-start_time):0.4f} FPS")
        # If the user presses the key 'q',
        # the program finishes
        if cv2.waitKey(1) & 0xFF ==ord('q'):
            print("\nProgram interrupted by the user - bye mate!")
            break
  """      



