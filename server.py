import cv2
import numpy as np
import paho.mqtt.client as mqtt

# Настройка MQTT
mqtt_broker = "localhost"  # Замените на IP сервера, если Mosquitto на другом устройстве
client = mqtt.Client()
client.connect(mqtt_broker)
client.loop_start()

# Захват видео с камеры
cap = cv2.VideoCapture(0)  # 0 для камеры по умолчанию (USB)
if not cap.isOpened():
    print("Ошибка: Не удалось открыть камеру.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Ошибка: Не удалось прочитать кадр.")
        break

    # Преобразование в HSV для обнаружения цвета
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([70, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Отправка автоматической команды
    if contours:
        client.publish("robot/automatic_command", "move_forward")
    else:
        client.publish("robot/automatic_command", "stop")

    # Отображение кадра
    cv2.imshow('Video', frame)

    # Проверка нажатия клавиш для ручных команд
    key = cv2.waitKey(10) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('a'):
        client.publish("robot/manual_command", "left")
    elif key == ord('d'):
        client.publish("robot/manual_command", "right")
    elif key == ord('w'):
        client.publish("robot/manual_command", "forward")
    elif key == ord('s'):
        client.publish("robot/manual_command", "backward")
    elif key == ord(' '):
        client.publish("robot/manual_command", "stop")

# Очистка ресурсов
cap.release()
cv2.destroyAllWindows()
client.loop_stop()