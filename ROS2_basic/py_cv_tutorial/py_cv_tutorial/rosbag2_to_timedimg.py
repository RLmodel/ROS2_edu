import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import cv2

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()

        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]

def main(args=None):

    # Change below roots to your ros2 bag locations
    ROOT_DIR = "/home/byb76/Downloads/quadrupped_train"
    FILENAME = "/quadrupped_train.bag_0.db3"
    DESCRIPTION = "color_"

    BAGFILE = ROOT_DIR + FILENAME
    print(BAGFILE)

    parser = BagFileParser(BAGFILE)

    # (timestamp, actual_data) => 301개
    img_topics = parser.get_messages("/camera/color/image_raw")

    for k, b in enumerate(img_topics):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(b[1], 'bgr8')
        cv_image.astype(np.uint8)
        if (DESCRIPTION == 'depth_'):
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET)
            cv2.imwrite(ROOT_DIR + '/depth/' + str(b[0]) + '.png', cv_image)
        else:
            cv2.imwrite(ROOT_DIR + '/color/' + str(b[0]) + '.png', cv_image)
        print('saved: ' + DESCRIPTION + str(b[0]) + '.png')
  
if __name__ == '__main__':
    main()