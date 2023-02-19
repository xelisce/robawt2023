import datetime

class FPS:
    def __init__(self):
        self.__start = None
        self.__end = None
        self.__numFrames = 0

    def start(self):
        self.__start = datetime.datetime.now()
        return self

    def stop(self):
        self.__end = datetime.datetime.now()
    
    def update(self):
        self.__numFrames += 1
    
    def elapsed(self):
        return (self.__end - self.__start).total_seconds()
    
    def fps(self):
        return self.__numFrames / self.elapsed()