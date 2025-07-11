import cv2
import numpy as np


lower1 = np.array([343, 69.5, 36.1])
upper1 = np.array([360, 119.5, 106.1])

lower2 = np.array([0, 69.5, 36.1])
upper2 = np.array([10, 119.5, 106.1])

cam = cv2.VideoCapture(0)   

while True:
    berhasil, gambar = cam.read()   
    
    if not berhasil:
        print("Kamera error!")
        break
    
    gambar = cv2.flip(gambar, 1)
    gambar_hsv = cv2.cvtColor(gambar, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(gambar_hsv, lower1, upper1)
    mask2 = cv2.inRange(gambar_hsv, lower2, upper2)
    mask = cv2.bitwise_xor(mask1, mask2)

    kernel1 = np.ones((5,5 ),np.uint8)
    erosion = cv2.erode(mask,kernel1,iterations = 1)
    dilation = cv2.dilate(erosion,kernel1,iterations = 1)


    kernel2 = np.ones((10,10 ),np.uint8)
    dilation2 = cv2.dilate(dilation,kernel2,iterations = 1)
    erosion2 = cv2.erode(dilation2,kernel2,iterations = 1)

    cv2.imshow("Mask ", erosion2)
    cv2.imshow("Maewwfsk ", dilation)
    cv2.imshow("nfnawifewa", mask)
    cv2.imshow("Kamera", gambar)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
print("Program selesai!")