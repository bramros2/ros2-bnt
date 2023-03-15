import rclpy
import sys
import cv2
import time


from rclpy.node             import Node
from std_msgs.msg           import String
from std_msgs.msg           import Float64
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from .flow_detector         import *

class DropletDetector(Node):


    def __init__(self,thr_min,thr_max,blur =15, blob_params=None, detection_window=None):
        super().__init__('droplet_detector')
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window
        
        self._t0 = time.time()
        
    
    

        print (">> Publishing image to topic image/flow")
        print(">> Publishing mask to topic image/mask")
        self.image_pub = self.create_publisher(Image, "image/flow", 1)
        self.mask_pub = self.create_publisher(Image, "image/mask", 1) 
        print (">> Publishing droplet size to topic droplet_size")
        self.droplet_pub = self.create_publisher(Float64, "droplet_size", 1)


        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, "/image_raw", self.callback, 1)
        print ("<< Subscribed to topic /image_raw")


    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]
        
    def set_blur(self, blur):
        self._blur = blur
      
    def set_blob_params(self, blob_params):
        self._blob_params = blob_params

    def callback(self,data):
        self.get_logger().info("Recieving image from topic image/raw")
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #cv2.imshow("Camera", cv_image)
            #cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            #--- Detect blobs
            keypoints, mask   = blob_detect(cv_image, self._threshold[0], self._threshold[1], self._blur,
                                            blob_params=self._blob_params, search_window=self.detection_window)
            #--- Draw search window and blobs
            cv_image    = blur_outside(cv_image, 10, self.detection_window)

            cv_image    = draw_window(cv_image, self.detection_window, line=1)
            cv_image    = draw_frame(cv_image)
            
            cv_image    = draw_keypoints(cv_image, keypoints) 
            try:
                cv2.imshow("Frame overlay", cv_image)
                cv2.imshow("Mask", mask)
                cv2.waitKey(1)
                #self.image_pub.publish(cv_image)
                self.get_logger().info("Publishing mask & image")
                mask_message = self.bridge.cv2_to_imgmsg(mask, encoding="passthrough")
                self.mask_pub.publish(mask_message)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)

        result = cv_image.copy()
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]
        if len(contours) != 0:
            # draw in blue the contours that were founded
            cv2.drawContours(result, contours, -1, 255, 3)

            # find the biggest countour (c) by the area
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)

            # draw the biggest contour (c) in green
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # show thresh and result
        cv2.imshow("bounding_box", cv_image)
        cv2.waitKey(0)

        msgdata = 'w = %s h = %s' % (w, h)
        self.get_logger().info(" %s " % (msgdata))

        average_wh = (w+h)/2
        droplet = Float64()
        droplet.data = average_wh
        self.droplet_pub.publish(droplet)

        fps = 1.0/(time.time()-self._t0)
        self._t0 = time.time()

def main(args=None):
    if args is None:
        args = sys.argv
    
    with open("/home/bram/ros2_ws/src/py_nodes/py_nodes/config.txt", 'r') as file:
        config_dict = {}
        for line in file:
            if line[0] != '#':  #ignores comments
                line.strip('\n')
                line = line.split()
                for elem in line:
                    if ord(elem[0]) >= 48 and ord(elem[0]) <= 57: #checks for digits
                        value = int(elem)                   #MAKE SURE NO FLOATS ARE NEEDED FOR VALUES IN SETTINGS
                    elif elem != '=':    #ignores the = sign
                        identifier = elem
                config_dict[identifier] = value
    hsv_min = (config_dict['hmin'],config_dict['smin'],config_dict['vmin'])
    hsv_max = (config_dict['hmax'],config_dict['smax'],config_dict['vmax'])

    blur = config_dict['blur']
    min_size = config_dict['min_size']
    max_size = config_dict['max_size']

    #--- detection window respect to camera frame in [x_min, y_min, x_max, y_max] adimensional (0 to 1)
    x_min   = 0.1
    x_max   = 0.9
    y_min   = 0.1
    y_max   = 0.9

    detection_window = [x_min, y_min, x_max, y_max]

    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 100;
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 20000
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.2
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.7   

    rclpy.init(args=args)

    drop_detector = DropletDetector(hsv_min, hsv_max, blur, params, detection_window)
    
    rclpy.spin(drop_detector)

    drop_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



