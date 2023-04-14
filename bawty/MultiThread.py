import cv2
from threading import Thread

class WebcamStream:
    
    def __init__(self, stream_id = 0):
        self.stream_id = stream_id
        print(self.stream_id)
        self.vcap = cv2.VideoCapture(self.stream_id)
        self.vcap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # assert self.vcap.isOpened()
        
        if self.vcap.isOpened() is False:
            print('[Exiting]: Error accessing webcam stream. (MultiThread)')
            exit(0)

        #print fps
        # fps_input_stream = int(self.vcap.get(5))
        # print('FPS of input stream:', fps_input_stream)

        #reading a single frame from vcap stream for initialisation
        self.grabbed, self.frame = self.vcap.read()
        if self.grabbed is False:
            print('[Exiting]: No more frames to read. (MultiThread)')
            exit(0)
        self.prevframe = self.frame
        #self.stopped is initialised to False
        self.stopped = True
        #thread instantiation
        self.t = Thread(target=self.update, args=())
        self.t.daemon = True #daemon threads run in the background

    def start(self):
        self.stopped = False
        self.t.start()
    
    def update(self):
        while True:
            # if self.stopped is True:
            #     break
            self.prevframe = self.frame
            self.grabbed, self.frame = self.vcap.read()
            if self.grabbed is False:
                print('[ERROR]: No more frames to read. (MultiThread)')
                # self.stopped = True
                # break
                self.frame = self.prevframe
            # else:
        self.vcap.release()
    
    def read(self):
        return self.frame
    
    def stop(self):
        self.stopped = True