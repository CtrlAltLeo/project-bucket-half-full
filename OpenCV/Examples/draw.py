import cv2 as cv
import numpy as np

blank = np.zeros((500,500,3), dtype='uint8')
cv.imshow("Blank", blank)

# Paint a certain part
blank[200:400, 400:500] = 0,255,0

cv.imshow("Green", blank)

# Rect
cv.rectangle(blank, (0,0), (250,250), (0,255,0))
cv.imshow("Rect", blank)


cv.waitKey(0)
