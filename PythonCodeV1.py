import cv2
import serial
import time
from simple_pid import PID

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

# Open the serial connection to the Arduino
arduino = serial.Serial('COM9', 9600)
time.sleep(2)

# Initialize the camera
cap = cv2.VideoCapture(1)

# Load the cascade
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Initialize the servo angles
pan_angle = 90
tilt_angle = 60
arduino.write(f'{pan_angle},{tilt_angle}\n'.encode())


# Initialize the PID
pan_pid = PID(0.4, 0.05, 0.05, setpoint=pan_angle) #0.35, 0.05, 0.05
pan_pid.output_limits = (-5, 5)
tilt_pid = PID(0.08, 0.01, 0.01, setpoint=tilt_angle) #0.07, 0.01, 0.01
tilt_pid.output_limits = (-5, 5)
pan_pid.sample_time = 0.01
tilt_pid.sample_time = 0.01

while True:
    # Read a frame from the camera
    ret, img = cap.read()

    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect faces in the image
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    # Draw a rectangle around each face
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Calculate the center of the face
        center_x = x + w / 2
        center_y = y + h / 2

        # Calculate the difference between the center of the face and the center of the frame
        x_diff = img.shape[1] / 2 - center_x
        y_diff = img.shape[0] / 2 - center_y
        # Convert pixels to degrees
        x_diff_deg = abs(x_diff / 15)
        y_diff_deg = abs(y_diff / 9)

        # Update setpoints with face position and create deadzone
        if x_diff < -30:
            pan_pid.setpoint = pan_angle - x_diff_deg
            pan_update = pan_pid(pan_angle)
        elif x_diff > 30:
            pan_pid.setpoint = pan_angle + x_diff_deg
            pan_update = pan_pid(pan_angle)
        else:
            print("Deadzone X")
            pan_update = 0
        if y_diff < -30:
            tilt_pid.setpoint = tilt_angle + y_diff_deg
            tilt_update = tilt_pid(tilt_angle)
        elif y_diff > 30:
            tilt_pid.setpoint = tilt_angle - y_diff_deg
            tilt_update = tilt_pid(tilt_angle)
        else:
            print("Deadzone Y")
            tilt_update = 0

        pan_angle += pan_update
        tilt_angle += tilt_update
        pan_angle = clamp(pan_angle, 0, 180)
        tilt_angle = clamp(tilt_angle, 30, 100)


    # Send the angles to the Arduino
    arduino.write(f'{pan_angle},{tilt_angle}\n'.encode())
    print("x = ", pan_angle, "y = ", tilt_angle)

    # Display the image
    cv2.imshow('img', img)

    # Check for the 'q' key to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the serial connection
cap.release()
arduino.close()
cv2.destroyAllWindows()


#Clamp the angle values sent to the arduino
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
