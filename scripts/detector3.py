import cv2
import argparse
import numpy as np
import rospy
from std_msgs.msg import Float64, Float64MultiArray


weight = '/home/nvidia/catkin_ws/src/catchit3d/scripts/yolov3/yolo-drone_110000_49.weights'
config = '/home/nvidia/catkin_ws/src/catchit3d/scripts/yolov3/yolo-drone.cfg'
video_source = '/home/nvidia/catkin_ws/src/catchit3d/test-footage/A1.mp4' #0 
class_path = '/home/nvidia/catkin_ws/src/catchit3d/scripts/yolov3/yolo-drone.txt'

disp_size = (1280,720) #(1280,720) #(640,360)
cap = cv2.VideoCapture(video_source)
result = cv2.VideoWriter('../results/results.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 30, disp_size)

with open(class_path, 'r') as f:
    classes = [line.strip() for line in f.readlines()]
    
COLORS = np.random.uniform(0, 255, size=(len(classes), 3))
net = cv2.dnn.readNet(weight, config)
scale = 0.00392
conf_threshold = 0.5
nms_threshold = 0.4


def get_output_layers(net):
    
    layer_names = net.getLayerNames()
    try:
        output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
    except:
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    return output_layers


def draw_prediction(img, class_id, confidence, x, y, x_plus_w, y_plus_h):

    label = str(classes[class_id])

    color = COLORS[class_id]

    cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)

    cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    
def main():    
    rospy.init_node("detector")
    camera_pub= rospy.Publisher('/observer/camera_coord', Float64MultiArray, queue_size=10)   
    rate = rospy.Rate(15)
    
    while cap.isOpened() and not rospy.is_shutdown():     
    
        # Read a frame from the video
        success, frame = cap.read()

        if success:
        
            Width = frame.shape[1]
            Height = frame.shape[0]
            blob = cv2.dnn.blobFromImage(frame, scale, (416,416), (0,0,0), True, crop=False)

            net.setInput(blob)

            outs = net.forward(get_output_layers(net))

            class_ids = []
            confidences = []
            boxes = []

            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        center_x = int(detection[0] * Width)
                        center_y = int(detection[1] * Height)
                        w = int(detection[2] * Width)
                        h = int(detection[3] * Height)
                        x = center_x - w / 2
                        y = center_y - h / 2
                        class_ids.append(class_id)
                        confidences.append(float(confidence))
                        boxes.append([x, y, w, h])


            indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
            if indices:
            
                for i in indices:
                    try:
                        box = boxes[i]
                    except:
                        i = i[0]
                        box = boxes[i]
                    
                    x = box[0]
                    y = box[1]
                    w = box[2]
                    h = box[3]
                    draw_prediction(frame, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))
                    camera_pub.publish(Float64MultiArray(data = (x,y,w,h,1)))
            
            else:
                camera_pub.publish(Float64MultiArray(data=[0,0,0,0,0]))
        
        
            cv2.imshow("object detection", frame)
            result.write(frame)
        else:
            camera_pub.publish(Float64MultiArray(data=[0,0,0,0,0]))
            break
        
        rate.sleep()
        
    cap.release()
    result.release()
    cv2.destroyAllWindows()
    print('Stopped by user')
    
if __name__ == "__main__":  
    main()



