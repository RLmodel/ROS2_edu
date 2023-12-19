# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - RLmodel ybbaek
# - https://www.rlmodel.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import matplotlib.pyplot as plt




class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/video_frames', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    
    # Display image
    gray = cv2.cvtColor(current_frame,1)
    result = process_frame(gray)
    # edge = cv2.Canny(gray, 0, 300)
    cv2.imshow("camera", result)
    
    cv2.waitKey(1)


    
def process_frame(frame):
    # 이미지 크기를 줄여 속도 향상
    height, width = frame.shape[:2]
    frame = cv2.resize(frame, (width // 2, height // 2))

    # 이미지를 HSV로 변환
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 노란색 차선을 위한 HSV 범위 설정
    lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([30, 255, 255], dtype=np.uint8)

    # 노란색 차선을 마스크로 추출
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 차선을 찾기 위한 그레이 스케일 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 가우시안 블러 적용
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Canny 엣지 검출
    edges = cv2.Canny(blurred, 50, 150)

    # 노란색 차선과 엣지를 합친 이미지 생성
    combined = cv2.bitwise_or(edges, yellow_mask)

    # 허프 변환을 사용하여 선 감지
    lines = cv2.HoughLinesP(combined, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=100)

    # 차선이 없는 경우 처리
    if lines is None:
        print("No lanes detected")
        return frame

    # 차선 그리기
    draw_lines(frame, lines)

    return frame

def draw_lines(frame, lines):
    # 왼쪽 차선과 오른쪽 차선을 구분하기 위한 리스트 초기화
    left_lines = []
    right_lines = []

    rnum = 0
    lnum = 0

    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1)

        # 기울기에 따라 왼쪽 또는 오른쪽 차선으로 분류
        if slope < 0:
            if ( -5< slope < -0.2 ):
                left_lines.append(line)
                lnum +=1
        else:
            if (0.2 <slope < 5):
                right_lines.append(line)
                rnum += 1

        print('rnum :',rnum)
        print('lnum : ',lnum)

    # 왼쪽 차선과 오른쪽 차선을 각각 그리기
    draw_line_segments(frame, left_lines, color=(0, 255, 0))
    draw_line_segments(frame, right_lines, color=(0, 0, 255))

    # 중앙선 계산 및 표시
    # center_line = calculate_center_line(left_lines, right_lines)
    # cv2.line(frame, center_line[0], center_line[1], (255, 255, 255), 2)

    # 픽셀 값 출력
    # pixel_value = calculate_pixel_value(frame, center_line)
    # cv2.putText(frame, f"Pixel Value: {pixel_value}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

def draw_line_segments(frame, lines, color):
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(frame, (x1, y1), (x2, y2), color, 2)

def calculate_center_line(left_lines, right_lines):
    # 왼쪽 차선과 오른쪽 차선의 끝점을 사용하여 중앙선 계산
    left_x = np.mean([line[0][0] for line in left_lines])
    left_y = np.mean([line[0][1] for line in left_lines])
    right_x = np.mean([line[0][2] for line in right_lines])
    right_y = np.mean([line[0][3] for line in right_lines])

    center_line = ((int(left_x), int(left_y)), (int(right_x), int(right_y)))

    return center_line

def calculate_pixel_value(frame, center_line):
        # 중앙선 위의 픽셀 값 계산
    mid_x = (center_line[0][0] + center_line[1][0]) // 2
    mid_y = (center_line[0][1] + center_line[1][1]) // 2

    pixel_value = frame[mid_y, mid_x, 0]  # 블루 채널의 픽셀 값

    return pixel_value

  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
#   # Spin the node so the callback function is called.
#   rclpy.spin(image_subscriber)
  
#   # Destroy the node explicitly
#   # (optional - otherwise it will be done automatically
#   # when the garbage collector destroys the node object)
#   image_subscriber.destroy_node()
  
#   # Shutdown the ROS client library for Python
#   rclpy.shutdown()
  try:
       rclpy.spin(image_subscriber)
  except KeyboardInterrupt:
        image_subscriber.get_logger().info('==stop clean===')
  except BaseException:
        image_subscriber.get_logger().info('==exception===')
        raise
  finally:
        rclpy.shutdown()

if __name__ == '__main__':
  main()