import cv2

webcam = cv2.VideoCapture(0)

if not webcam.isOpened():
    print("Could not open webcam")
    exit()

while webcam.isOpened():
    status, frame = webcam.read()

    if status:
        cv2.putText(frame,"RLmodel ROS2 CV",(100,100),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)

        cv2.imshow("test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()
    