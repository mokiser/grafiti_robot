import cv2
import numpy as np
import paho.mqtt.client as mqtt
import time
import sys
import datetime

# Функция для логирования команд
def log_command(command, command_type, delta_angle=None, distance=None, fps=None):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    log_entry = f"[{timestamp}] {command_type} command: {command}"
    if delta_angle is not None:
        log_entry += f", delta_angle={np.degrees(delta_angle):.2f} degrees"
    if distance is not None:
        log_entry += f", distance_to_green={distance:.2f} pixels"
    if fps is not None:
        log_entry += f", FPS={fps:.1f}"
    with open("commands.log", "a") as f:
        f.write(log_entry + "\n")

# MQTT callback
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connected to MQTT broker")
    else:
        print(f"Failed to connect, return code: {rc}")

# Initialize MQTT client
client = mqtt.Client(client_id="server_client", callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
try:
    client.connect("192.168.1.103", 1883, 60)
except Exception as e:
    print(f"Error connecting to MQTT broker: {e}")
    sys.exit(1)
client.loop_start()

# Initialize camera
cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("Error: Could not open camera")
    client.loop_stop()
    client.disconnect()
    sys.exit(0)
cap.set(cv2.CAP_PROP_EXPOSURE, 0.5)

# Initialize command timing
last_auto_command_time = 0
last_manual_command_time = 0
reached_green = False
manual_command_interval = 0.1
last_turn_command = ""
last_turn_time = 0
last_red_center = None
last_blue_center = None
delta_angle_history = []

# Detect colored objects
def detect_objects(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Green
    lower_green = np.array([25, 50, 50])
    upper_green = np.array([95, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_green = cv2.erode(mask_green, None, iterations=3)
    mask_green = cv2.dilate(mask_green, None, iterations=3)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_center = None
    green_contour = None
    if contours_green:
        largest_green = max(contours_green, key=cv2.contourArea)
        if cv2.contourArea(largest_green) > 500:
            M_green = cv2.moments(largest_green)
            if M_green["m00"] != 0:
                green_center = (int(M_green["m10"] / M_green["m00"]), int(M_green["m01"] / M_green["m00"]))
                green_contour = largest_green

    # Red
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_center = None
    red_contour = None
    if contours_red:
        largest_red = max(contours_red, key=cv2.contourArea)
        if cv2.contourArea(largest_red) > 500:
            M_red = cv2.moments(largest_red)
            if M_red["m00"] != 0:
                red_center = (int(M_red["m10"] / M_red["m00"]), int(M_red["m01"] / M_red["m00"]))
                red_contour = largest_red

    # Blue
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([140, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_center = None
    blue_contour = None
    if contours_blue:
        largest_blue = max(contours_blue, key=cv2.contourArea)
        if cv2.contourArea(largest_blue) > 500:
            M_blue = cv2.moments(largest_blue)
            if M_blue["m00"] != 0:
                blue_center = (int(M_blue["m10"] / M_blue["m00"]), int(M_blue["m01"] / M_blue["m00"]))
                blue_contour = largest_blue

    # Стабилизация красного и синего центров
    global last_red_center, last_blue_center
    if red_center and last_red_center:
        if np.sqrt((red_center[0] - last_red_center[0])**2 + (red_center[1] - last_red_center[1])**2) > 100:
            red_center = last_red_center
    if blue_center and last_blue_center:
        if np.sqrt((blue_center[0] - last_blue_center[0])**2 + (blue_center[1] - last_blue_center[1])**2) > 100:
            blue_center = last_blue_center
    if red_center and blue_center:
        if np.sqrt((red_center[0] - blue_center[0])**2 + (red_center[1] - blue_center[1])**2) < 50:
            red_center = last_red_center
            blue_center = last_blue_center
    last_red_center = red_center
    last_blue_center = blue_center

    return green_contour, green_center, red_contour, red_center, blue_contour, blue_center, mask_green

# Automatic control
def automatic_control(frame):
    global last_auto_command_time, reached_green, last_turn_command, last_turn_time, delta_angle_history
    current_time = time.time()
    
    green_contour, green_center, red_contour, red_center, blue_contour, blue_center, mask_green = detect_objects(frame)
    
    # Draw contours and centers
    if green_contour is not None:
        x, y, w, h = cv2.boundingRect(green_contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "Green", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    if red_contour is not None:
        x, y, w, h = cv2.boundingRect(red_contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, "Red", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    if blue_contour is not None:
        x, y, w, h = cv2.boundingRect(blue_contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(frame, "Blue", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

    # Draw centers
    if green_center:
        cv2.circle(frame, green_center, 5, (0, 255, 0), -1)
    if red_center:
        cv2.circle(frame, red_center, 5, (0, 0, 255), -1)
    if blue_center:
        cv2.circle(frame, blue_center, 5, (255, 0, 0), -1)

    # Draw orientation line
    if red_center and blue_center:
        cv2.line(frame, blue_center, red_center, (255, 255, 0), 2)

    # Calculate robot position and orientation
    command = None
    delta_angle = 0
    distance_to_green = 0
    if green_center and red_center and blue_center and not reached_green:
        robot_x = (red_center[0] + blue_center[0]) / 2
        robot_y = (red_center[1] + blue_center[1]) / 2
        
        # Calculate distance to green
        distance_to_green = np.sqrt((green_center[0] - robot_x)**2 + (green_center[1] - robot_y)**2)
        distance_threshold = 100

        if distance_to_green <= distance_threshold:
            command = "stop"
            reached_green = True
            print(f"Reached green object (distance={distance_to_green:.2f} pixels), stopping")
        else:
            # Draw vector to green
            cv2.line(frame, (int(robot_x), int(robot_y)), green_center, (255, 255, 255), 2)

            dx_robot = red_center[0] - blue_center[0]
            dy_robot = red_center[1] - blue_center[1]
            robot_orientation = np.arctan2(dy_robot, dx_robot) if dx_robot != 0 or dy_robot != 0 else 0

            dx_green = green_center[0] - robot_x
            dy_green = green_center[1] - robot_y
            angle_to_green = np.arctan2(dy_green, dx_green)

            delta_angle = angle_to_green - robot_orientation
            if delta_angle > np.pi:
                delta_angle -= 2 * np.pi
            elif delta_angle < -np.pi:
                delta_angle += 2 * np.pi

            # Сглаживание delta_angle
            delta_angle_history.append(delta_angle)
            if len(delta_angle_history) > 5:
                delta_angle_history.pop(0)
            smoothed_delta_angle = np.mean(delta_angle_history)

            # Decide command with hysteresis
            threshold = np.radians(35)
            hysteresis = np.radians(20)
            if abs(smoothed_delta_angle) <= threshold:
                command = "forward"
            elif smoothed_delta_angle > 0 and (last_turn_command != "right" or abs(smoothed_delta_angle) > threshold + hysteresis):
                command = "right"
                last_turn_command = "right"
                last_turn_time = current_time
            elif smoothed_delta_angle < 0 and (last_turn_command != "left" or abs(smoothed_delta_angle) > threshold + hysteresis):
                command = "left"
                last_turn_command = "left"
                last_turn_time = current_time
            else:
                command = last_turn_command

            if command in ["left", "right"] and current_time - last_turn_time < 0.5:
                command = last_turn_command

    if command:
        if current_time - last_auto_command_time >= 0.25:
            client.publish("robot/gpio", command)
            fps = 1 / (time.time() - current_time)
            if command == "stop":
                print(f"Sent automatic command: {command} (distance={distance_to_green:.2f} pixels, FPS={fps:.1f})")
                log_command(command, "Automatic", distance=distance_to_green, fps=fps)
            else:
                print(f"Sent automatic command: {command} (delta_angle={np.degrees(smoothed_delta_angle):.2f} degrees, distance={distance_to_green:.2f} pixels, FPS={fps:.1f})")
                log_command(command, "Automatic", smoothed_delta_angle, distance_to_green, fps)
            last_auto_command_time = current_time
            time.sleep(0.1)
        else:
            fps = 1 / (time.time() - current_time)
            print(f"Automatic command skipped: {command} (waiting {0.25 - (current_time - last_auto_command_time):.1f}s, FPS={fps:.1f})")
    elif reached_green:
        fps = 1 / (time.time() - current_time)
        print(f"No automatic command sent (reached green object, FPS={fps:.1f})")
    else:
        fps = 1 / (time.time() - current_time)
        print(f"No automatic command sent (not all objects detected, FPS={fps:.1f})")

    cv2.imwrite("test_frame.jpg", frame) # Frame for debugging
    cv2.imshow("Green Mask", mask_green)
    return frame

# Manual control
def manual_control(key):
    global last_manual_command_time, reached_green
    current_time = time.time()
    
    manual_command = None
    if key == ord('w'):
        manual_command = "forward"
    elif key == ord('a'):
        manual_command = "left"
    elif key == ord('d'):
        manual_command = "right"
    elif key == ord('s'):
        manual_command = "backward"
    elif key == ord(' '):
        manual_command = "stop"
        reached_green = False
    
    if manual_command and current_time - last_manual_command_time >= manual_command_interval:
        client.publish("robot/gpio", manual_command)
        fps = 1 / (time.time() - current_time)
        print(f"Sent manual command: {manual_command}")
        log_command(manual_command, "Manual", fps=fps)
        last_manual_command_time = current_time
        time.sleep(0.1)
    elif manual_command:
        fps = 1 / (time.time() - current_time)
        print(f"Manual command skipped: {manual_command} (waiting {manual_command_interval - (current_time - last_manual_command_time):.1f}s)")


print("Press 'w' (forward), 'a' (left), 'd' (right), 's' (backward), 'space' (stop), 'q' (quit)")
# Main loop
while True:
    start_time = time.time()
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    frame = automatic_control(frame)
    cv2.imshow("Camera", frame)

    key = cv2.waitKey(10) & 0xFF
    if key == ord('q'):
        break
    manual_control(key)

# Cleanup
cap.release()
cv2.destroyAllWindows()
client.loop_stop()
client.disconnect()
