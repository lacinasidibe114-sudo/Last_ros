#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    """
    Node ROS 2 pour afficher les images de la caméra du robot.
    """
    
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Bridge pour convertir les messages ROS en images OpenCV
        self.bridge = CvBridge()
        
        # Abonnement au topic de la caméra
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Camera Viewer démarré - En attente des images...')
        
    def image_callback(self, msg):
        """
        Callback appelé à chaque nouvelle image reçue.
        """
        try:
            # Conversion du message ROS en image OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Affichage de l'image
            cv2.imshow('Camera Robot', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Erreur lors du traitement de l\'image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    camera_viewer = CameraViewer()
    
    try:
        rclpy.spin(camera_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        camera_viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()