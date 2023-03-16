import cv2
from MultiThread import WebcamStream
from FPS import FPS

#Note: FPS counts every frame processed, if looking for every frame passed must modify code

webcam_stream = WebcamStream(stream_id=0) #0 id for main camera, 2 for evac
webcam_stream.start()
timer = FPS().start()

while True:
    if webcam_stream.stopped is True:
        break
    else:
        frame = webcam_stream.read()
    
    cv2.imshow('frame', frame)
    timer.update()
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    
timer.stop()
print('FPS:', timer.fps(), 'Elapsed Time:', timer.elapsed())

webcam_stream.stop()
cv2.destroyAllWindows()
