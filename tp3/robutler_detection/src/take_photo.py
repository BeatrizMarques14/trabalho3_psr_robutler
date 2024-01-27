#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import os
from datetime import datetime

class PhotoTaker:
    def __init__(self):
        rospy.init_node('take_photo', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('robutler_detection') # Substitua 'robutler_detection' pelo nome do seu pacote

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Exibe a imagem
        cv2.imshow('Captured Image', cv_image)
        cv2.waitKey(0)  # Aguarda uma tecla ser pressionada
        cv2.destroyAllWindows()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_name = f"photo_{timestamp}.jpg"
        image_path = os.path.join(self.package_path, 'photos', image_name)

        # Salva a imagem
        cv2.imwrite(image_path, cv_image)
        rospy.loginfo("Foto capturada e salva como photo.jpg")

        # Depois de tirar a foto, desregistra o subscriber para não tirar mais fotos
        self.image_sub.unregister()

if __name__ == '__main__':
    photo_taker = PhotoTaker()
    rospy.spin()

