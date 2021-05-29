import cv2
import sys
sys.path.append('/home/pi/ArmPi/')
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import numpy as np

AK = ArmIK()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red',)
# Set detection color
def setTargetColor(target_color):
    global __target_color

    #print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

class Perception():

    def __init__(self):
        return

    # Find the contour with the largest area
    # The parameter is a list of contours to be compared
    @staticmethod
    def getAreaMaxContour(contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # Traverse all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # Only when the area is greater than 300，The contour with the largest area is effective，To filter out interference
                    area_max_contour = c

        return area_max_contour, contour_area_max  # Return the largest contour

    def draw_plus(self, img):
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        return img

    def preprocess(self, img, get_roi, roi, start_pick_up):

        size = (640, 480)

        frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

        #If a recognized object is detected in a certain area，Then keep detecting the area until there is no
        if get_roi and start_pick_up:
            get_roi = False
            frame_gb = getMaskROI(frame_gb, roi, size)

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB space

        return frame_lab

    def detect_color_contour(self, frame_lab, color_range):

        frame_mask = cv2.inRange(frame_lab, color_range[0], color_range[1])  # Perform bit operations on the original image and mask

        # isolate detecteced "blobs" then find maximum region
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Open operation
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # Closed operation
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find the outline
        areaMaxContour, area_max = self.getAreaMaxContour(contours)  # Find the largest contour

        return areaMaxContour, area_max

    def label_img(self, img, box, world_x, world_y, color):
        cv2.drawContours(img, [box], -1, color, 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1) #Draw center point
        return img


range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red',)


# set detection color
def setTargetColor(target_color):
    global __target_color

    # print("COLOR", target_color)
    __target_color = target_color
    return (True, ())


# The angle at which the gripper is closed when gripping
servo1 = 500


# initial position
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)


def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)


# Set the RGB light color of the expansion board to make it consistent with the color to be tracked
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()


count = 0
track = False
_stop = False
get_roi = False
center_list = []
first_move = True
__isRunning = False
detect_color = 'None'
action_finish = True
start_pick_up = False
start_count_t1 = True


# Variable reset
def reset():
    global count
    global track
    global _stop
    global get_roi
    global first_move
    global center_list
    global __isRunning
    global detect_color
    global action_finish
    global start_pick_up
    global __target_color
    global start_count_t1

    count = 0
    _stop = False
    track = False
    get_roi = False
    center_list = []
    first_move = True
    __target_color = ()
    detect_color = 'None'
    action_finish = True
    start_pick_up = False
    start_count_t1 = True


# app initialization call
def init():
    print("ColorTracking Init")


# App start playing method call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")


# app stop gameplay call
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Stop")


# App exit gameplay call
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Exit")


rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
world_x, world_y = 0, 0

t1 = 0
roi = ()
last_x, last_y = 0, 0


def run(img):
    global roi
    global rect
    global count
    global track
    global get_roi
    global center_list
    global __isRunning
    global unreachable
    global detect_color
    global action_finish
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global world_x, world_y
    global start_count_t1, t1
    global start_pick_up, first_move

    img_utils = ImageUtils()

    img_copy = img_utils.draw_plus(img.copy())

    if not __isRunning:
        return img

    frame_lab = img_utils.preprocess(img_copy, get_roi, roi, start_pick_up)

    area_max = 0
    areaMaxContour = 0
    if not start_pick_up:  # if we haven't started picking stuff up yet
        for i in color_range:  # for each color in possible colors
            if i in __target_color:
                detect_color = i

                ######################### Detect a specific color ######################################
                # in:  frame_lab, color_range
                # out: areaMaxContour, area_max
                areaMaxContour, area_max = img_utils.detect_color_contour(frame_lab, color_range[detect_color])
                ########################################################################################

        # only use areas of sufficient size (determined by this magic number)
        if area_max > 2500:  # Have found the largest area

            ############## Contour to Box ################3
            # in:  areaMaxContour
            # out: box
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))
            ########################################

            get_roi = True
            ############ Box to world coordinates ################
            # in:  box
            # out: roi
            roi = getROI(box)  # Get roi area

            img_centerx, img_centery = getCenter(rect, roi, size,
                                                 square_length)  # Get the center coordinates of the block
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size)  # Convert to real world coordinates
            ####################################################

            img = img_utils.label_img(img, box, world_x, world_y, range_rgb[detect_color])

            # Compare to last coordinate to determine if we have to move
            distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2))
            last_x, last_y = world_x, world_y
            track = True
            # print(count,distance)

            # Cumulative judgment
            if action_finish:
                if distance < 0.3:
                    center_list.extend((world_x, world_y))
                    count += 1
                    if start_count_t1:
                        start_count_t1 = False
                        t1 = time.time()
                    if time.time() - t1 > 1.5:
                        rotation_angle = rect[2]
                        start_count_t1 = True
                        world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                        count = 0
                        center_list = []
                        start_pick_up = True
                else:
                    t1 = time.time()
                    start_count_t1 = True
                    count = 0
                    center_list = []
    return img

if __name__ == '__main__':
    init()
    start()
    __target_color = ('red', )
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()