import sys
from os.path import expanduser
import time
import math
from multiprocessing import Process, Pipe
import cv2
import datetime

class MyWebcam:

    def __init__(self):
        # get image resolution
        self.img_width = 640
        self.img_height = 480

        # get image center
        self.img_center_x = self.img_width / 2
        self.img_center_y = self.img_height / 2
        
        # define field of view
        self.cam_hfov = 70.42
        self.cam_vfov = 43.3

        # background image processing variables
        self.proc = None            # background process object
        self.parent_conn = None     # parent end of communicatoin pipe
        self.img_counter = 0        # num images requested so far

    # get_camera - initialises camera and returns VideoCapture object 
    def get_camera(self):
        # setup video capture
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,self.img_width)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,self.img_height)

        # check we can connect to camera
        if not self.camera.isOpened():
            print "failed to open camera, exiting!"
            sys.exit(0)

        return self.camera

    # image_capture_background - captures all images from the camera in the background and returning the latest image via the pipe when the parent requests it
    def image_capture_background(self, imgcap_connection):
        # exit immediately if imgcap_connection is invalid
        if imgcap_connection is None:
            print "image_capture failed because pipe is uninitialised"
            return

        # open the camera
        camera = self.get_camera()

        # clear latest image
        latest_image = None

        while True:
            # constantly get the image from the webcam
            success_flag, image=camera.read()

            # if successful overwrite our latest image
            if success_flag:
                latest_image = image

            filename = "webcam photo " + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S") + ".jpg"
            # write the latest image into the video - BlueZhong
            cv2.imwrite(filename, image)

            # check if the parent wants the image
            if imgcap_connection.poll():
                recv_obj = imgcap_connection.recv()
                # if -1 is received we exit
                if recv_obj == -1:
                    break

                # otherwise we return the latest image
                imgcap_connection.send(latest_image)

        # release camera when exiting
        camera.release()

    # start_background_capture - starts background image capture
    def start_background_capture(self):
        # create pipe
        self.parent_conn, imgcap_conn = Pipe()

        # create and start the sub process and pass it it's end of the pipe
        self.proc = Process(target=self.image_capture_background, args=(imgcap_conn,))
        self.proc.start()

    def stop_background_capture(self):
        # send exit command to image capture process
        self.parent_conn.send(-1)

        # join process
        self.proc.join()

    # get_image - returns latest image from the camera captured from the background process
    def get_image(self):
        # return immediately if pipe is not initialised
        if self.parent_conn == None:
            return None

        # send request to image capture for image
        self.parent_conn.send(self.img_counter)

        # increment counter for next interation
        self.img_counter = self.img_counter + 1

        # wait endlessly until image is returned
        recv_img = self.parent_conn.recv()

        # return image to caller
        return recv_img

    # main - tests BalloonVideo class
    def main(self):

        # start background process
        self.start_background_capture()

        while True:
            # send request to image capture for image
            img = self.get_image()
    
            # check image is valid
            if not img is None:
                # display image
                cv2.imshow ('image_display', img)
            else:
                print "no image"
    
            # check for ESC key being pressed
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
    
            # take a rest for a bit
            time.sleep(0.1)

        # send exit command to image capture process
        self.stop_background_capture()

        print "a2p 10 = %f" % self.angle_to_pixels_x(10)
        print "p2a 10 = %f" % self.pixels_to_angle_x(10)
        self.video_writer = self.open_video_writer()

# create a single global object
my_webcam = MyWebcam()

if __name__ == "__main__":
    my_webcam.main()
