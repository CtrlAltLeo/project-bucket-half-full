import cv2 as cv

capture = cv.VideoCapture(0)

IsTrue, frame = capture.read()
cv.imshow("Cam", frame)

blur = cv.GaussianBlur(frame, (11,11), cv.BORDER_DEFAULT)
cv.imshow("Blur", blur)

edge1 = cv.Canny(frame, 50, 50)
edge2 = cv.Canny(blur, 50, 50)

cv.imshow("e1", edge1)
cv.imshow("e2", edge2)

cv.waitKey(0)

#while True:
#  isTrue, frame = capture.read()
#  cv.imshow("Webcam", frame)
