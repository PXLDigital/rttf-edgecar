import numpy as np

''' 
    Fast implementation of a naive CV algorithm
    Steers away from nearest detected border
    Input: binary image with track borders in white
'''

class Scanner:

    def __init__(self):
        self.DELTA_STEP = 2

    def set_image(self, img):
        self.img = img
        self.width = np.shape(img)[1]
        self.height = np.shape(img)[0]      

    # steer away from nearest border
    def direction(self):
        left = 0
        right = 0

        for i in range(self.height-1, 0, -1):
            row = self.img[i]
            if left == 0 and row[0] == 255:
                left = i
            if right == 0 and row[self.width-1] == 255:
                right = i
            if not left == 0 and not right == 0:
                break

        target = None
        absdir = None
        if abs(left-right) < 8:
            target = (0, int(self.width /2))
            absdir = 0
        elif left > right: # go right
            target = (right, self.width)
            absdir = 1
        elif right > left: # go left
            target = (left, 0)
            absdir = -1

        return (target, absdir, left, right)

    def find_direction(self):
        current_row = int(self.height * 0.5) # 1/2 of screen height
        limit = int(self.height * 0.25) # 1/4 of screen height

        left = 0
        right = self.width
        current_row += self.DELTA_STEP # yes i know it's ugly, leave me alone
        while current_row > limit and (left == 0 or right == self.width):
            current_row -= self.DELTA_STEP
            (left, right) = self.scan_row(current_row)
        
        mid = (right + left) / 2
        screenmid = self.width / 2
        direction = (mid - screenmid) / screenmid 
        # negative = left, postive = right

        return (direction, int(mid), int(current_row), left, right)

    def scan_row(self, r):
        row = self.img[r]
        screenmid = self.width / 2

        left = None
        right = None
        for (i, px) in enumerate(row):
            if px == 255:
                if i < screenmid:
                    if left == None or left == -1:
                        left = -1
                elif right == None:
                    right = i
                    break
            
            elif px == 0 and left == -1 and i < screenmid:
                left = i-1

        if left == None or left == -1:
            left = 0
        if right == None:
            right = self.width

        return (left, right)
        