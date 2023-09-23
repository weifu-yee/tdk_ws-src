import rospy
from std_msgs.msg import Int32MultiArray
from ultralytics import YOLO
import argparse


def main():
    # receive command-ling arguments
    parser = argparse.ArgumentParser(description='Process idx argument.')
    parser.add_argument('idx', type=int, help='an integer argument')
    args = parser.parse_args()
    
    rospy.init_node('detect_node')
    pub = rospy.Publisher('/numbers', Int32MultiArray, queue_size=10)

    model = YOLO("/home/ditrobotics/tdk_ws/src/tutorial/src/number.pt")
    img = f"/home/ditrobotics/tdk_ws/src/tutorial/src/capture_{args.idx}.jpg"  # img path
    results = model.predict(source=img, save=True)

    # Get detected class name
    cls_name = []
    for r in results:
        cls_name.extend(r.boxes.boxes[:, -1].tolist())

    msg = Int32MultiArray()
    msg.data = [int(x + 1) for x in cls_name]
    
    pub.publish(msg)
    rospy.loginfo("Published detected class: %s", msg.data)
    rospy.sleep(1) # ensure publish
    

if __name__ == '__main__':
    main()
