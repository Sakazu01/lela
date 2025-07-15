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


skala = 10

waktu_jatuh, jarak_horizontal = calculate_drop_data(9, 9)
jarak_pixel = int(jarak_horizontal * skala)