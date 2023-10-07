#!/usr/bin/python
import rospy
from std_msgs.msg import Int32MultiArray, Bool
from ultralytics import YOLO

# ROS init
pub = rospy.Publisher('/numbers', Int32MultiArray, queue_size=10)
msg = Int32MultiArray()

model = YOLO("/home/ditrobotics/tdk_ws/src/yolov8/src/number.pt")

prev = False
def detect_callback(msg):
    global prev
    if msg.data and not prev:
        detect_img()
    prev = msg.data

def detect_img():
    global pub, msg
    print("[YOLO] start inference ...")
    img_path = f"/home/ditrobotics/tdk_ws/src/yolov8/jpg/capture.jpg"
    results = model.predict(source=img_path, save=True)
    rospy.loginfo("[3] Detect Finished")
  
    # Get detected class name
    names = model.names
    cls_name = []
    
    for r in results:
        for c in r.boxes.cls:
            cls_name.append(names[int(c)])        
    print(cls_name)
    
    # update msg
    msg.data = [int(x + 1) for x in cls_name]
    
    rospy.loginfo("[3] Published detected class: %s", msg.data)
    pub.publish(msg)
    

def main():
    rospy.init_node('detect_node')
    rospy.Subscriber('/detect', Bool, detect_callback)
    rospy.spin() 

if __name__ == '__main__':
    main()
