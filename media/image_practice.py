#!/usr/bin/env python

import sys
import math
import cv2
import numpy as np
import argparse
import os
import scipy.ndimage as ndi
from PIL import Image


def main(argv):
    
    parser = argparse.ArgumentParser(description="Get some clicks")
    parser.add_argument("input_path", help="Path to input image file")

    args = parser.parse_args()
    input_file = args.input_path

    if not os.path.isfile(input_file):
        print("Error, need an input file.")
        sys.exit(1)
    
    # Loads an image and applies a mask to isolate white pixels
    image = cv2.imread(input_file, cv2.IMREAD_COLOR)
    lower = np.array([180, 180, 180], dtype="uint8")
    upper = np.array([255, 255, 255], dtype="uint8")
    mask = cv2.inRange(image, lower, upper)
    masked = cv2. bitwise_and(image, image, mask = mask)
    im = Image.fromarray(masked).convert('RGB')
    im.save('masked.png')

    src = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
    # src = cv2.imread(masked, cv2.IMREAD_GRAYSCALE)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
        return -1

    dst = cv2.Canny(src[0:], 50, 200, None, 3)
    
    # Copy edges to the images that will display the results in BGR
    cdstP = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    
    # Finds Hough Lines
    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

    # # cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    im = Image.fromarray(cdstP).convert('RGB')
    im.save('masked_hough_lines.png')
    
    sys.exit(0)
    return 0
    
if __name__ == "__main__":
    main(sys.argv[1:])
