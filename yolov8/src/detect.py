#!/usr/bin/python
import rospy
from rospy import Rate
from std_msgs.msg import Int32MultiArray, Bool
from ultralytics import YOLO
import os


# ROS init
pub = rospy.Publisher('/numbers', Int32MultiArray, queue_size=10)
msg = Int32MultiArray()

model = YOLO("/home/ditrobotics/tdk_ws/src/yolov8/src/best.pt")

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
    results = model.predict(source=img_path, save=True, conf=0.7)
    rospy.loginfo("[3] Detect Finished")
  
    # Get detected class name
    cls_name = []
    
    for r in results:
        for c in r.boxes.cls:
            cls_name.append(int(c))
    
    # update msg
    msg.data = [int(x + 1) for x in cls_name]

    
    # 20 Hz
    rate = Rate(20)
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < 1.0 and not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
    
    rospy.loginfo("[3] Published detected class: %s", msg.data)
    
    # Delete the jpg
    # if os.path.exists(img_path):
    #     os.remove(img_path)
    #     rospy.loginfo(f"[detect.py] {img_path} has been deleted.")
    

def main():
    rospy.init_node('detect_node')
    rospy.Subscriber('/detect', Bool, detect_callback)
    rospy.loginfo("\n\n!!!!!!!!!! [detect.py] initialized !!!!!!!!!!!!!\n\n")
    rospy.spin() 

if __name__ == '__main__':
    main()
