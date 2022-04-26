#!/usr/bin/env python

import sys
import math
import cv2
import numpy as np
import argparse
import os
from PIL import Image


def main(argv):
    
    parser = argparse.ArgumentParser(description="Get some clicks")
    parser.add_argument("input_path", help="Path to input image file")

    args = parser.parse_args()
    input_file = args.input_path

    if not os.path.isfile(input_file):
        print("Error, need an input file.")
        sys.exit(1)
    
    # Loads an image
    src = cv2.imread(input_file, cv2.IMREAD_GRAYSCALE)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
        return -1
    
    
    dst = cv2.Canny(src[0:], 50, 200, None, 3)
    
    # Copy edges to the images that will display the results in BGR
    cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    
    # lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
    
    # if lines is not None:
    #     for i in range(0, len(lines)):
    #         rho = lines[i][0][0]
    #         theta = lines[i][0][1]
    #         a = math.cos(theta)
    #         b = math.sin(theta)
    #         x0 = a * rho
    #         y0 = b * rho
    #         pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
    #         pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
    #         cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    
    
    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
    
    # # cv2.imshow("Source", src)
    # im = Image.fromarray(src).convert('RGB')
    # im.save('Source.png')

    # # cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
    # im = Image.fromarray(cdst).convert('RGB')
    # im.save('CDST.png')

    # cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    im = Image.fromarray(cdstP).convert('RGB')
    im.save('CDSTP.png')
    
    cv2.waitKey()
    return 0
    
if __name__ == "__main__":
    main(sys.argv[1:])
