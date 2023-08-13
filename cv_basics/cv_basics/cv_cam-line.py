import cv2

webcam = cv2.VideoCapture(0)

if not webcam.isOpened():
    print("Could not open webcam")
    exit()

while webcam.isOpened():
    status, frame = webcam.read()

    if status:
        cv2.line(frame,(100,500),(300,300),(0,255,0),5)
        cv2.line(frame,(400,300),(600,500),(0,0,255),5)

        cv2.imshow("test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()
    