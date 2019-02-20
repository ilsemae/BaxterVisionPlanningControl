def pixelfield()
    #reads 4 images, take the images by key press or time initiated
    top = cv2.imread('GET TOP IMAGE')
    bottom = cv2.imread('GET BOTTOM IMAGE')
    left = cv2.imread('GET LEFT IMAGE')
    right = cv2.imread('GET RIGHT IMAGE')
    images = [top,bottom,left,right]
    
    # pixboard = [(top(x,y,z), bottom(x,y,z), left(x,y,z), right(x,y,z)]
    pixboard = np.zeros( (4,3) )
    
    i = 0
    #image = cv2.imread(images[1])
    for image in images:                    
        hsv_img = cv2.GaussianBlur(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),(5, 5), 0)

        lower = color_boundaries[0][0]
        upper = color_boundaries[0][1]
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
         
        # find the colors within the specified boundaries and apply the mask
        mask   = cv2.inRange(hsv_img, lower, upper)
        output = cv2.bitwise_and(hsv_img, hsv_img, mask = mask)

        #cv2.imshow("images", np.hstack([image, hsv_img, output]))
        #cv2.waitKey(0)

        img = cv2.medianBlur(output,5)
        cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        #find circles in image
        kernel = np.ones((5,5),np.uint8)
        kernel[0,0] = 0
        kernel[0,4] = 0
        kernel[4,0] = 0
        kernel[4,4] = 0
        dilated = cv2.dilate(cimg,kernel,iterations=1)
        closing = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel)
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
        circles = cv2.HoughCircles(closing,cv2.cv.CV_HOUGH_GRADIENT,1,20,param1=25,param2=15,minRadius=5,maxRadius=11)
        ret,thresh1=cv2.threshold(opening,127,255,cv2.THRESH_BINARY)
        (r,c)=thresh1.shape
        
        (top_x,top_y)=np.nonzero(thresh1)
       
            
        pixboard[i][0] = circles[0][0] #column value
        pixboard[i][1] = circles[0][1] #row value
        i = i + 1
    return pixboard

