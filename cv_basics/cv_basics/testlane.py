

import cv2
import numpy as np
import matplotlib.pyplot as plt

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked_img = cv2.bitwise_and(img, mask)
    return masked_img

def draw_lines(img, lines, color=(255, 0, 0), thickness=2):
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def display_lines(img, lines):
    line_img = np.zeros_like(img)
    draw_lines(line_img, lines)
    return cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

def process_image(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise and help with edge detection
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Use Canny edge detection
    edges = cv2.Canny(blur, 50, 150)

    # Define region of interest (ROI)
    height, width = edges.shape
    vertices = np.array([[(0, height), (width/2, height/2), (width, height)]], dtype=np.int32)
    roi_edges = region_of_interest(edges, vertices)

    # Use HoughLinesP to detect lines in the image
    lines = cv2.HoughLinesP(roi_edges, 2, np.pi/180, threshold=50, minLineLength=100, maxLineGap=50)

    if lines is not None:
        # Display lines on the original image
        line_img = display_lines(image, lines)

        # Find the center of the two main lanes
        lane_centers = [((x1 + x2) // 2, (y1 + y2) // 2) for line in lines for x1, y1, x2, y2 in line]
        main_lane_center = np.mean(lane_centers, axis=0, dtype=int)

        # Draw a vertical line at the center of the two main lanes
        cv2.line(line_img, (main_lane_center[0], 0), (main_lane_center[0], height), (0, 255, 0), 2)

        # Display the pixel location text on the screen
        cv2.putText(line_img, f"Center: {main_lane_center[0]} pixels", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        return line_img
    else:
        return image  # If no lines are detected, return the original image

# Open a video capture object (0 corresponds to the default camera)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Process the frame
    result = process_image(frame)

    # Display the result
    cv2.imshow('Lane Detection', result)

    # Break the loop when 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()