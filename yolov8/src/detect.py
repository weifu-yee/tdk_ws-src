#!/usr/bin/python3
import rospy
from std_msgs.msg import Int32MultiArray, Bool
from ultralytics import YOLO

# ROS init
pub = rospy.Publisher('/numbers', Int32MultiArray, queue_size=10)
msg = Int32MultiArray()

model = YOLO("/home/ditrobotics/tdk_ws/src/yolov8/src/number.pt")

def detect_callback(msg):
    print('received')
    if msg.data:  # Check if message data is True
        detect_img()

def detect_img():
    print("[YOLO] start inference ...")
    img_path = f"/home/ditrobotics/tdk_ws/src/yolov8/jpg/capture.jpg"
    results = model.predict(source=img_path, save=True)

    # Get detected class name
    cls_name = []
    for r in results:
        cls_name.extend(r.boxes.boxes[:, -1].tolist())

    # update msg
    msg.data = [int(x + 1) for x in cls_name]
    pub.publish(msg)
    rospy.loginfo("Published detected class: %s", msg.data)

def main():
    rospy.init_node('detect_node')
    rospy.Subscriber('/detect', Bool, detect_callback)  # Create subscriber
    rospy.spin()  # Keep the node running until it's shut down

if __name__ == '__main__':
    main()
