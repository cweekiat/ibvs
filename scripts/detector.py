#! /usr/bin/env python

import cv2
from ultralytics import YOLO
import rospy
from std_msgs.msg import Float64, Float64MultiArray

# TODO
# 1. Change to max 1 detection per frame
# 2. Add a moving avg filter
# 3. A script for yolov3 weight?


# Load the YOLOv8 model
model_path = '/home/nvidia/catkin_ws/src/catchit3d/models/custom.pt'
model = YOLO(model_path)


# Open the video file
video_source = 0#'/home/nvidia/catkin_ws/src/catchit3d/test-footage/dji4_720.mp4' #0 

disp_size = (640,360) #(1280,720) #(640,360)
cap = cv2.VideoCapture(video_source)
out = cv2.VideoWriter('../results/results.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 30, disp_size)



def main():
    rospy.init_node("detector")
    camera_pub= rospy.Publisher('/observer/camera_coord', Float64MultiArray, queue_size=10)   
    rate = rospy.Rate(15)
    
    # Loop through the video frames
    while cap.isOpened() and not rospy.is_shutdown():     
    
        # Read a frame from the video
        success, frame = cap.read()

        if success:
            # Run YOLOv8 inference on the frame
            results = model.predict(frame, max_det =1)

            boxes = results[0].boxes
            if boxes:
                box = boxes[0]
                data = box.xywh.tolist()[0]
                data.append(1)
                camera_pub.publish(Float64MultiArray(data=data))
            else:
                camera_pub.publish(Float64MultiArray(data=[0,0,0,0,0]))
                        
            # Visualize the results on the frame
            annotated_frame = results[0].plot()
            out.write(annotated_frame)

            # Display the annotated frame
            # cv2.imshow("YOLOv8 Inference", annotated_frame)
        else:
            # Break the loop if the end of the video is reached
            camera_pub.publish(Float64MultiArray(data=[0,0,0,0,0]))
            break
            
        rate.sleep()
           
    # Release the video capture object and close the display window
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print('Stopped by user')
            

if __name__ == "__main__":  
    main()


