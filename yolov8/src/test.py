#!/usr/bin/python3

from ultralytics import YOLO

model = YOLO("/home/ditrobotics/tdk_ws/src/yolov8/src/number.pt")
img_path = f"/home/ditrobotics/tdk_ws/src/yolov8/jpg/capture.jpg"

print("[YOLO] start inference ...")
results = model.predict(source=img_path, save=True)

print(results)