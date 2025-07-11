import cv2
import numpy as np

def calculate_drop_data(altitude, horizontal_speed, gravitasi = 9.8):
    time_to_fall = np.sqrt((2 * altitude) / gravitasi)
    horizontal_distance = horizontal_speed * time_to_fall
    return time_to_fall, horizontal_distance

def get_user_input():
    altitude = float(input("altitude (meter): "))
    horizontal_speed = float(input("Horizontal speed (m/s): "))
    payload_weight = float(input("payload (gram): "))
    return altitude, horizontal_speed, payload_weight

lower1 = np.array([343, 69.5, 36.1])
upper1 = np.array([360, 119.5, 106.1])

lower2 = np.array([0, 69.5, 36.1])
upper2 = np.array([10, 119.5, 106.1])

cam = cv2.VideoCapture(0)

# tinggi, kecepatan, berat = get_user_input()
tinggi = 0
kecepatan = 0
berat = 0
waktu_jatuh, jarak_horizontal = calculate_drop_data(9, 9)
# waktu_jatuh, jarak_horizontal = calculate_drop_data(altitude, horizontal_speed) 

tinggi = 0

skala = 10  
jarak_pixel = int(jarak_horizontal * skala)

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

    bentuk_objek, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(bentuk_objek) > 0:
        for objek in bentuk_objek:
            if cv2.contourArea(objek) > 500:
                x, y, lebar, tinggi_obj = cv2.boundingRect(objek)
                cv2.rectangle(gambar, (x, y), (x + lebar, y + tinggi_obj), (0, 0, 255), 3)
                
                tengah_x = x + lebar // 2
                tengah_y = y + tinggi_obj // 2
                
                cv2.circle(gambar, (tengah_x, tengah_y), 5, (255, 0, 0), -1)
                cv2.putText(gambar, f"({tengah_x}, {tengah_y})", 
                           (tengah_x + 10, tengah_y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                titik_jatuh_x = tengah_x + jarak_pixel
                titik_jatuh_y = tengah_y

                cv2.circle(gambar, (titik_jatuh_x, titik_jatuh_y), 5, (0, 255, 0), -1)
                cv2.putText(gambar, "Titik Jatuh", 
                           (titik_jatuh_x + 10, titik_jatuh_y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    cv2.putText(gambar, f"Tinggi: {tinggi} m", (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(gambar, f"Kecepatan: {kecepatan} m/s", (10, 60), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(gambar, f"Berat: {berat} g", (10, 90), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    

    cv2.imshow("Mask ", erosion2)
    cv2.imshow("Maewwfsk ", dilation)
    cv2.imshow("nfnawifewa", mask)
    cv2.imshow("Kamera", gambar)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
print("Program selesai!")