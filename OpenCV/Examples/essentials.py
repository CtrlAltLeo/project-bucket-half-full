import cv2 as cv

# Grayscale
img = cv.imread("ORB.png")
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

cv.imshow("orb", cv.resize(img, (250,250), cv.INTER_AREA))
cv.imshow("gray", gray)

# Blur
blur = cv.GaussianBlur(img, (11,11), cv.BORDER_DEFAULT)
cv.imshow("Blur", blur)



# Edge Cascade
canny = cv.Canny(img, 50, 70)
cv.imshow("Edge Detectoin", canny)

# Combining Edge with Blur to reduce noise
canny1 = cv.Canny(blur, 50, 70)
cv.imshow("Edge Cascade w/ Blur", canny1)



cv.waitKey(0)
