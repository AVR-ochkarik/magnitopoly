import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import threading

"""  КОНСТАНТЫ  """
#   порты
A = 1; B = 2
#   настройки кадра и усреднения
FRAMES_FOR_DETECTION = 5
CAMERA_INDEX = 0
DP = 1
MIN_DIST = 20
PARAM1 = 52
PARAM2 = 28
MIN_RADIUS = 1
MAX_RADIUS = 30
#   текст 
font = cv2.FONT_HERSHEY_SIMPLEX
org = (70, 70)
fontScale = 1
color = (0, 0, 255)
thickness = 2
"""  КОНСТАНТЫ\  """

# флаги
trigger = False
busy = False
# настройка портов
GPIO.setmode(GPIO.BCM)
GPIO.setup(A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(B, GPIO.OUT, initial=GPIO.LOW)

def input_callback(channel):
    """по факту просто ставит флаг срабатывания"""
    global trigger
    if GPIO.input(channel): trigger = True

# обработчик фронта
GPIO.add_event_detect(A, GPIO.RISING, callback=input_callback, bouncetime=200)

def detect_circles(cap, frames=FRAMES_FOR_DETECTION):
    """для каждого из frames кадров считаем кружки, усредняем, возвращаем"""
    averager = []
    last_image_for_display = None
    for i in range(frames):
        ret, img = cap.read()
        # если не прочиталось
        if not ret:
            time.sleep(0.1)
            continue
        image = cv2.flip(img, 1) 
        last_image_for_display = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_blured = cv2.blur(gray, (3, 3))

        detected_circles = cv2.HoughCircles(gray_blured, cv2.HOUGH_GRADIENT, DP, MIN_DIST,
                                            param1=PARAM1, param2=PARAM2,
                                            minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)
        # кол-во кругов в этом кадре
        if detected_circles is not None: count = len(detected_circles[0])
        else: count = 0
        averager.append(count)
        time.sleep(0.05) # мини-тайм-аут камере

    if len(averager) == 0: average = 0
    else: average = round(sum(averager) / len(averager))
    if average < 1: average = 1
    if average > 6: average = 6
    # текст 
    if last_image_for_display is not None:
        cv2.putText(image, str(average), org, font, fontScale, color, thickness, cv2.LINE_AA)
        cv2.rectangle(image, (60, 40), (100, 80), color, thickness)
        cv2.imshow("Detected Circle", image)
    return average

def pulse_output(count, duration=1.0): 
    """count импульсов: B -> HIGH на короткое время, затем LOW"""
    global busy
    busy = True
    try:
        if count <= 0:
            busy = False
            return
        period = duration / count
        # можно сделать ширину импульса размеренной типа min(0.1, period / 2) будет условно 100 миллисекунд или половина периода
        pulse_width = 0.05
        for i in range(count):
            GPIO.output(B, GPIO.HIGH)
            time.sleep(pulse_width)
            GPIO.output(B, GPIO.LOW)
            # оставшееся время до следующего начала
            rem = period - pulse_width
            if rem > 0: time.sleep(rem)
    finally: busy = False

def main_loop():
    global trigger, busy
    camera_stream = cv2.VideoCapture(CAMERA_INDEX)
    # обработка если камера не открыта
    if not camera_stream.isOpened(): return
    last_detected = "-"

    try:
        while True:
            ret, image = camera_stream.read()
            # если не прочиталось
            if not ret:
                time.sleep(0.1)
                continue
            image = cv2.flip(image, 1)

            if trigger and not busy:
                trigger = False
                detected_count = detect_circles(camera_stream, frames=FRAMES_FOR_DETECTION)
                last_detected = str(detected_count)
                thread = threading.Thread(target=pulse_output, args=(detected_count, 1), daemon=True)
                thread.start()
            try: avg_show = int(last_detected) if last_detected != "-" else "-"
            except: avg_show = "-"
            image = cv2.putText(image, str(avg_show), org, font, fontScale, color, thickness, cv2.LINE_AA)
            cv2.rectangle(image, (60, 40), (160, 80), color, thickness)
            cv2.imshow("Detected Circle", image)

            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                # выход по нажатию "q"
                break

    except KeyboardInterrupt:
        # выход из-за KeyboardInterrupt
        a = 1
    finally:
        camera_stream.release()
        cv2.destroyAllWindows()
if __name__ == "__main__":
    try: main_loop()

    finally: GPIO.cleanup()
