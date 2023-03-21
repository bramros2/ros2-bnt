import cv2
import time

from flow_detector import *

class DropletDetector:
    def __init__(self, thr_min, thr_max, blur=15, blob_params=None, detection_window=None):
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window
        self._t0 = time.time()

    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]

    def set_blur(self, blur):
        self._blur = blur

    def set_blob_params(self, blob_params):
        self._blob_params = blob_params

    def run(self, img_path):
        cv_image = cv2.imread(img_path)
        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            # --- Detect blobs
            keypoints, mask = blob_detect(cv_image, self._threshold[0], self._threshold[1], self._blur,
                                          blob_params=self._blob_params, search_window=self.detection_window)
            # --- Draw search window and blobs
            cv_image = blur_outside(cv_image, 10, self.detection_window)
            cv_image = draw_window(cv_image, self.detection_window, line=1)
            cv_image = draw_frame(cv_image)
            cv_image = draw_keypoints(cv_image, keypoints)

            cv2.imshow("Frame overlay", cv_image)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)

        result = cv_image.copy()
        for kp in keypoints:
            x, y = kp.pt
            r = kp.size / 2
            x, y, r = int(x), int(y), int(r)
            cv2.circle(result, (x, y), r, (0, 255, 0), thickness=2)
            print(kp, kp.size)

        cv2.imshow("result", result)
        cv2.waitKey(0)


        fps = 1.0 / (time.time() - self._t0)
        self._t0 = time.time()


def main(args=None):
    hsv_min = 0,0,194
    hsv_max = 255,255,255
    blur = 0
    x_min = 0
    x_max = 1
    y_min = 0
    y_max = 1

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

    drop_detector = DropletDetector(hsv_min, hsv_max, blur, params, detection_window)
    path = '/Users/bramo/Downloads/droplet.jpg'
    drop_detector.run(path)

if __name__ == '__main__':
    main()

