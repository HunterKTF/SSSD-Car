"""
sdc_library.py

Description: Library with all the functions needed for the lane line detection

Author: Yosthin Galindo
Contact: yosthin.galindo@udem.edu
First created: Monday 27 october, 2022
Usage:

"""


# Import required libraries

import numpy as np
import cv2
import math
import matplotlib.pyplot as plt



def input_image(img_name, result):
    """
    This function reads the input image and opens it in a new window
    
    Inputs:
    img_name: name of the image to be read

    Outputs: 
    img_colour: image read by opencv and reduced to one fourth of the original size

    """

    #Verify that image exists
    try:
        # Be advissed that cv2.IMREAD_REDUCED_COLOR_4 reduces the
        # image size by one-fourth
        #img_colour = cv2.imread(img_name)
        
        ret, frame = img_name.read()
    except:
        print('ERROR')
        exit()
    
    result.write(frame)
    cv2.imshow("Colour image", frame)
    return frame

def greyscale_img(img_colour):
    """
    This function takes the image read before and turn it into greyscale
    
    Inputs:
    img_colour: image with RGB color

    Outputs: 
    grey: image read turned into greyscale

    """

    grey = cv2.cvtColor(img_colour, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("Greyscale image", grey) 
    return grey

def smoothed_img(grey, kernel_size):
    """
    This function takes the greyscale image and smooth it with a function called Gaussian blur.
    
    Inputs:
    grey : image in greyscale
    kernel_size: size of the kernel which the image will be smoothed with

    Outputs: 
    blur_grey: image with blur

    """

    blur_grey = cv2.GaussianBlur(grey, kernel_size, sigmaX=0, sigmaY=0)
    #cv2.imshow("smoothed image", blur_grey)
    return blur_grey

def canny_img(blur_grey, low_threshold, high_threshold):
    """
    This function detect the edges of the image
    
    Inputs:
    blur_grey: image with blur
    low_threshold: low range of the threshold needed for the Canny function
    high_threshold: high range of the threshold needed for the Canny function

    Outputs: 
    edges: image in black and white with the detected edges in white

    """
    
    edges = cv2.Canny(blur_grey, low_threshold, high_threshold, apertureSize=3)
    
    #cv2.imshow("Canny image", edges)
    return edges

#Get a region of interest
def region_of_interest(img, vertices):
    """
    This function gets the region of interest of the image where we want to detect the lanes
    
    Inputs:
    img: image with detected edges
    vertices: vertices of the polygon area where we are putting the region of interest

    Outputs: 
    masked_image: image with detected edges only in the region of interest

    """
    
    #mask with 0
    mask = np.copy(img)*0
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(img, mask)
    cv2.imshow("Canny image within Region of Interest", masked_image)
    return masked_image

def warpImg(img, points, w, h):
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    matrix = cv2.getPerspectiveTransform(pts1,pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w,h))
    cv2.imshow("Warped img", imgWarp)
    return imgWarp

def nothing(a):
    pass

def initializeTrackbars(initialTrackbarVals, wT, hT):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", initialTrackbarVals[0], wT//2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", initialTrackbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", initialTrackbarVals[2], wT//2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", initialTrackbarVals[3], hT, nothing)

def valTrackbars(wT, hT):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                        (widthBottom, heightBottom), (wT-widthBottom, heightBottom)])
    return points

def drawPoints(img, points):
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 15, (0,0,255), cv2.FILLED)

    cv2.imshow("Points", img)
    return img



def getHistogram(img, minValue=0.1, display=False):

    histValues = np.sum(img, axis=0)
    #print(f"histogram: {histValues}")
    maxValue = np.max(histValues)

    indexArray = np.where(histValues>=minValue)
    basePoint = int(np.average(indexArray))
    #print(f"basepoint: {basePoint}")

    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1],3), np.uint8)
        for x, intensity in enumerate(histValues):
            #print(f"instensity:{intensity}")
            cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0]-intensity//255), (255,0,255),1)
            cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0,255,255), cv2.FILLED)
        cv2.imshow("Histogram", imgHist)
    return basePoint



def hough(img_colour, roi_image, rho, theta, threshold, min_line_len, max_line_gap, prev_x1_hough, prev_y1_hough, prev_x2_hough, prev_y2_hough):
    """
    This function gets hough lines with the given parameters and the function HoughLinesP
    
    Inputs:
    img_colour: image with RGB color
    roi_image: image with the region of interest detected edges
    rho: distance resolution in pixels of the Hough grid
    theta: angular resolution in radians of the Hough grid
    threshold: minimum number of votes (intersections in Hough grid)
    min_line_len: minimum number of pixels making up a line
    max_line_gap: maximum gap in pixels between connectable line segments

    Outputs: 
    hough lines: image with hough lines

    """

    img_colour_with_lines = img_colour.copy()
    try:
        hough_lines = cv2.HoughLinesP(roi_image, rho, theta, threshold, np.array([]),
                                    minLineLength=min_line_len, maxLineGap=max_line_gap)
        for line in hough_lines:
            for x1, y1, x2, y2 in line:
                prev_x1_hough.append(x1)
                prev_y1_hough.append(y1)
                prev_x2_hough.append(x2)
                prev_y2_hough.append(y2)
                cv2.line(img_colour_with_lines, (x1, y1), (x2, y2), (255,0,0), 5)
    except:
        cv2.line(img_colour_with_lines, (prev_x1_hough[-1], prev_y1_hough[-1]), (prev_x2_hough[-1], prev_y2_hough[-1]), (255,0,0), 5)
        
    #result.write(img_colour_with_lines)
    #cv2.imshow('Hough lines', img_colour_with_lines)
    return hough_lines, prev_x1_hough, prev_y1_hough, prev_x2_hough, prev_y2_hough

def left_and_right_lines(hough_lines, img_colour):
    """
    This function draws the left and right lines
    
    Inputs:
    img_colour: image with RGB color
    hough_lines: image with hough lines

    Outputs: 
    left_line_x: left points of lines in x axis
    left_line_y: left points of lines in y axis
    right_line_x: right points of lines in x axis
    right_line_y: right points of lines in y axis


    """

    img_colour_with_left_and_right_lines = img_colour.copy()
    
    #lines arrays
    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []
    try:
        #get slope
        for line in hough_lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) #slope
                if math.fabs(slope) < 0.2: #Only consider extreme slope
                    continue  
                
                if slope <= 0: #Negative slope, left group.
                    if x1 <= 400 and x2 <= 400: #only consider the left lines in the left side of the image
                        left_line_x.extend([x1, x2])
                        left_line_y.extend([y1, y2])
                        cv2.line(img_colour_with_left_and_right_lines, (x1, y1), (x2, y2), (0,255,0), 5) #draw left line
                        
                else: #Otherwise, right group.
                    if x1 > 250 and x2 > 250: #only consider the right lines in the right side of the image
                        right_line_x.extend([x1, x2])
                        right_line_y.extend([y1, y2])
                        cv2.line(img_colour_with_left_and_right_lines, (x1, y1), (x2, y2), (0,0,255), 5) #draw right line
            
    except:
        print("ERROR! No hough lines detected")
    #cv2.imshow('Left and right lines', img_colour_with_left_and_right_lines)
    return left_line_x, left_line_y, right_line_x, right_line_y
    
def lane_lines(left_line_x, left_line_y, right_line_x, right_line_y, img_colour, result2, prev_x1_r, prev_y1_r, prev_x2_r, prev_y2_r, prev_x1_l, prev_y1_l, prev_x2_l, prev_y2_l):
    """
    This function draws the lane lines
    
    Inputs:
    left_line_x: left points of lines in x axis
    left_line_y: left points of lines in y axis
    right_line_x: right points of lines in x axis
    right_line_y: right points of lines in y axis
    img_colour: image with RGB color

    Outputs: 
    define_lines: points of the lane lines 


    """

    img_lane_lines = img_colour.copy()
    #min and max of the line
    min_y = 200
    max_y = 500
    
    if len(left_line_x)>0 and len(left_line_y)>0:
        
        #Create a function that match with all the detected lines
        poly_left = np.poly1d(np.polyfit(
            left_line_y,
            left_line_x,
            deg=1
        ))
        #get the start and the end
        left_x_start = int(poly_left(max_y))
        left_x_end = int(poly_left(min_y))
    
        #save points
        
        left_lines=[[
                [left_x_start, max_y, left_x_end, min_y],
            ]]  
        
        #Add both lines
        for line in left_lines:
            for x1, y1, x2, y2 in line:
                prev_x1_l.append(x1)
                prev_y1_l.append(y1)
                prev_x2_l.append(x2)
                prev_y2_l.append(y2)
                cv2.line(img_lane_lines, (x1, y1), (x2, y2), (255,0,0), 12)
                

    else:
        cv2.line(img_lane_lines, (prev_x1_l[-1], prev_y1_l[-1]), (prev_x2_l[-1], prev_y2_l[-1]), (255,0,0), 12)
        #print("ERROR! No left lane lines detected") 
        
    if len(right_line_x)>0 and len(right_line_y)>0:
    
        
        
        #Create a function that match with all the detected lines
        poly_right = np.poly1d(np.polyfit(
            right_line_y,
            right_line_x,
            deg=1
        ))
        #get the start and the end
        right_x_start = int(poly_right(max_y))
        right_x_end = int(poly_right(min_y))

        #save points
        right_lines=[[
                [right_x_start, max_y, right_x_end, min_y],
            ]]  
        
        #Add both lines
        for line in right_lines:
            for x1, y1, x2, y2 in line:
                prev_x1_r.append(x1)
                prev_y1_r.append(y1)
                prev_x2_r.append(x2)
                prev_y2_r.append(y2)
                cv2.line(img_lane_lines, (x1, y1), (x2, y2), (255,0,0), 12)
        

    else:
        cv2.line(img_lane_lines, (prev_x1_r[-1], prev_y1_r[-1]), (prev_x2_r[-1], prev_y2_r[-1]), (255,0,0), 12)
        #print("ERROR! No right lane lines detected")

    if len(right_line_x)>0 and len(right_line_y)>0 and len(left_line_x)>0 and len(left_line_y)>0:
        pts = np.array([[left_x_start, max_y],[right_x_start, max_y], [right_x_end, min_y], [left_x_end, min_y]], np.int32)
        pts = pts.reshape((-1,1,2))
        cv2.fillPoly(img_lane_lines, [pts], (255,255,255))
    else:
        pts = np.array([[prev_x1_l[-1], prev_y1_l[-1]],[prev_x1_r[-1], prev_y1_r[-1]], 
                        [prev_x2_r[-1], prev_y2_r[-1]], [prev_x2_l[-1], prev_y2_l[-1]]], np.int32)
        pts = pts.reshape((-1,1,2))
        cv2.fillPoly(img_lane_lines, [pts], (255,255,255))

    
    result2.write(img_lane_lines)
    cv2.imshow('LINES', img_lane_lines)
    return img_lane_lines, prev_x1_r, prev_y1_r, prev_x2_r, prev_y2_r, prev_x1_l, prev_y1_l, prev_x2_l, prev_y2_l