import cv2 as cv
import pandas as pd

cv.namedWindow("output", cv.WINDOW_NORMAL)    # Create window with freedom of dimensions
image = cv.imread("1.bmp")                    # Read image
#imS = cv.resize(im, (960, 540))                # Resize image
#cv.imshow("output", im)                       # Show image
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
parameters =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)
markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)
cv.waitKey(0)                 


# cameraMatrixLeft = [2416.134291, 2416.134291, 0, 1246.104594, 0, 2416.904917, 1006.702368, 0, 0, 1]
# distortionCoeffsLeft = [-0.0019, -0.0019, 2.50E-05, 7.50E-05, -1.10E-05, 9.50E-08, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# cameraMatrixRight = [2427.256144, 2427.256144, 0, 1238, 0, 2430.44497, 1013, 0, 0, 1]
# distortionCoeffRight = [-0.002,	-0.002,	4.50E-05, 1.80E-05, -3.70E-05, -4.20E-07, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# termCriteria = [0.0001,	0.0001,	100, 'Iter | Eps']
# stereoRotation = [0.984104809, 0.984104809, 0.028004222, 0.175366726, -0.027150103, 0.999604944, -0.007268268, -0.175500989, 0.002391513, 0.98447635]
# stereoTranslation = [-860.461, -860.461, 10.754, 77.236]


# cv.stereoCalibrate()