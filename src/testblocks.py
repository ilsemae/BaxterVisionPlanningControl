import numpy as np
import cv2

def run():
    lower = np.array([103,  97, 100])
    upper = np.array([120, 255, 255])
    img = cv2.imread('/home/cs4752/ros_ws/src/group7_final/src/frame0000.jpg')
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_img = cv2.GaussianBlur(cv2.cvtColor(img,cv2.COLOR_BGR2HSV),(5, 5), 0)

    mask = cv2.inRange(hsv_img, lower, upper)

    res = cv2.bitwise_and(img,img, mask=mask)
    cv2.imwrite('res.jpg',res)
    ngray = cv2.cvtColor(res,cv2.COLOR_HSV2BGR)
    ngray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
    kernel = np.ones((2.5,2.5),np.uint8)
    opening = cv2.morphologyEx(ngray, cv2.MORPH_OPEN, kernel)

    kernel = np.ones((3,3),np.uint8)
    closing = cv2.morphologyEx(ngray, cv2.MORPH_CLOSE, kernel)
    cv2.imshow("images", np.hstack([ngray,opening,cv2.bitwise_not(closing)]))
    cv2.waitKey(0)

    params = cv2.SimpleBlobDetector_Params()

    #params.filterByColor = 1
    #params.blobColor = 0
    #params.maxThreshold = 90
    #params.filterByCircularity = 1
    #params.maxCircularity = .5
    #params.filterByArea = 1
    #params.minArea = 5

    #params.filterByConvexity = 1
    #params.minConvexity = .5

    
    #closing  = cv2.dilate(closing,kernel,iterations=2)
    inverted = cv2.bitwise_not(closing)
    ret,binary = cv2.threshold(inverted,150,255,cv2.THRESH_BINARY)
    detector = cv2.SimpleBlobDetector(params)
    keypoints = detector.detect(binary)
    print(img.shape)
    shape = np.array(img.shape)
    print()
    for key in keypoints:
        print(key.pt)
    im = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow("images", np.hstack([inverted,closing,binary]))
    cv2.waitKey(0)
    cv2.imshow("drawing", np.hstack([im]))
    cv2.waitKey(0)
    


run()
