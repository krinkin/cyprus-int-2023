
### Code:

#set color HSV
self.HSV = {
"yellow": [np.array([11, 115, 70]), np.array([40, 255, 245])],
"red": [np.array([0, 43, 46]), np.array([8, 255, 255])],
"green": [np.array([35, 43, 46]), np.array([77, 255, 255])], # [77, 255, 255]
"blue": [np.array([100, 43, 46]), np.array([124, 255, 255])],
"cyan": [np.array([78, 43, 46]), np.array([99, 255, 255])], # np.array([78, 43, 46]), np.array([99, 255, 255])
}
def color_detect(self, img):
        # set the arrangement of color'HSV
        x = y = 0
        for mycolor, item in self.HSV.items():
            redLower = np.array(item[0])
            redUpper = np.array(item[1])
            # transfrom the img to model of gray
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # wipe off all color expect color in range
            mask = cv2.inRange(hsv, item[0], item[1])
            ...