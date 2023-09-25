#!/usr/bin/python3
import rospy
from std_msgs.msg import Int32MultiArray, Bool
from ultralytics import YOLO

# ROS init
pub = rospy.Publisher('/numbers', Int32MultiArray, queue_size=10)
msg = Int32MultiArray()

class Detector:
    def __init__(self):
        self.count = 0
        self.model = YOLO("/home/ditrobotics/tdk_ws/src/yolov8/src/number.pt")
        self.sub = rospy.Subscriber('/detect', Bool, self.detect_callback)  # Create subscriber

    def detect_callback(self, msg):
        if msg.data:  # Check if message data is True
            self.detect_img()

    def detect_img(self):
        print("[YOLO] start inference ...")
        img_path = f"/home/ditrobotics/tdk_ws/src/yolov8/jpg/capture.jpg"
        results = self.model.predict(source=img_path, save=True)

        # Get detected class name
        cls_name = []
        for r in results:
            cls_name.extend(r.boxes.boxes[:, -1].tolist())

        # update msg
        msg.data = [int(x + 1) for x in cls_name]
        pub.publish(msg)
        rospy.loginfo("Published detected class: %s", msg.data)
        
        # Increment count
        self.count += 1 

def main():
    rospy.init_node('detect_node')
    detector = Detector()
    rospy.spin()  # Keep the node running until it's shut down

if __name__ == '__main__':
    main()