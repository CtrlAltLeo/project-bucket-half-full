import cv2 as cv


def rescaleFrame(frame, scale=0.5):
  width = int(frame.shape[1] * scale)
  height = int(frame.shape[0] * scale)
  dimensions = (width, height)

  return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

# Image reading
img = cv.imread("./ORB.png")

scaled_img = rescaleFrame(img)

cv.imshow("Orb", scaled_img)

# Video Reading
#capture = cv.VideoCapture(0)

#while True:
#  isTrue, frame = capture.read()
#  cv.imshow("Live Footage", frame)


cv.waitKey(0)
