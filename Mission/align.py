import time
import cv2
import numpy as np

class KalmanFilter:
    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        # State: [x, y, vx, vy]
        self.kf.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kf.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        self.kf.processNoiseCov = 1e-4 * np.eye(4, dtype=np.float32)
    
    def update(self, meas_x, meas_y):
        self.kf.correct(np.array([[meas_x], [meas_y]], dtype=np.float32))
    
    def predict(self):
        pred = self.kf.predict()
        return pred[0], pred[1]

h_x = 320
h_y = 240
target = None

def mid_coord(x, y):
    global h_x, h_y
    h_x = int(x)
    h_y = int(y)

def clearFile(filename,num1, num2):
    with open(filename, "w") as f:
        f.write(f"{num1}, {num2}\n") 

def readingCoordinates(filename):
    with open(filename, "r") as file:
        lines = file.readlines()
        if not lines:
            return None, None  # Return None if the file is empty
        last_line = lines[-1].strip()
        x, y = map(int, last_line.split(", "))
        return x, y
    
def init_align(drone):
    global h_x, h_y, target
    print("Move Towards Target")
    clearFile("pixel.txt",h_x, h_y)
    x_coord = h_x
    y_coord = h_y
    
    for i in range(12):
        x_coord,y_coord = readingCoordinates("pixel.txt")
        time.sleep(0.5)
        print(f"points color={x_coord},{y_coord}")
        send_velocity_based_on_position(x_coord, y_coord, 0.28)
        time.sleep(1)

    time.sleep(1)
    target = drone.get_position()
    print(f"Target Coordinates: {target['latitude']}, {target['longitude']}")
    drone.take_picture()
    time.sleep(1)

def align_to_target(drone):
    clearFile("target_midpoint.txt",h_x, h_y)
    time.sleep(1)
    kf = KalmanFilter()
    while True:
        # Get current altitude
        current_alt = drone.get_altitude()

        # Exit condition
        x, y = readingCoordinates("target_midpoint.txt")  # Raw detection
        kf.update(x, y)
        pred_x, pred_y = kf.predict()
        
        # Use predicted coordinates for alignment
        error_x = pred_x - h_x
        error_y = pred_y - h_y
        
        # Velocity adjustments (existing code)
        send_velocity_based_on_position(pred_x, pred_y, 0.25)
        
        if abs(error_x) < 10 and abs(error_y) < 10:
            if current_alt <= 13:   # Descend to 10 m
                print(f"Reached target altitude: {current_alt:.2f} m")
                break
            else:
                drone.send_ned_velocity(0, 0, 0.25)
    time.sleep(2)   
    drone.drop_payload(2300)
    time.sleep(4)

def send_velocity_based_on_position(drone, x_coord,y_coord,g_speed,z_speed = 0):
    global h_x, h_y
    if x_coord == h_x and y_coord == h_y:
        drone.send_ned_velocity(0, 0, z_speed)
    elif x_coord > h_x and y_coord > h_y:
        drone.send_ned_velocity(-g_speed, g_speed, z_speed)
    elif x_coord < h_x and y_coord < h_y:
        drone.send_ned_velocity(g_speed, -g_speed, z_speed)
    elif x_coord < h_x and y_coord > h_y:
        drone.send_ned_velocity(-g_speed, -g_speed, z_speed)
    elif x_coord > h_x and y_coord < h_y:
        drone.send_ned_velocity(g_speed, g_speed, z_speed)
    elif x_coord == h_x and y_coord != h_y:
        if y_coord > h_y:
            drone.send_ned_velocity(0, -g_speed, z_speed)
        elif y_coord < h_y:
            drone.send_ned_velocity(0, g_speed, z_speed)
    elif y_coord == h_y and x_coord != h_x:
        if x_coord > h_x:
            drone.send_ned_velocity(g_speed, 0, z_speed)
        elif x_coord < h_x:
            drone.send_ned_velocity(-g_speed, 0, z_speed)