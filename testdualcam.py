import cv2
from MultiThread import WebcamStream

#camera initialisation & parameters
lt_stream = WebcamStream(stream_id=0) #0 id for main camera
lt_stream.start()

evac_stream = WebcamStream(stream_id=2) #0 id for main camera
evac_stream.start()

lt_frame = lt_stream.read()
evac_frame = evac_stream.read()

lt_dim = {'w': lt_frame.shape[1], 'h': lt_frame.shape[0]}
evac_dim = {'w': evac_frame.shape[1], 'h': evac_frame.shape[0]}

print("Line track camera width:", lt_dim['w'], "Camera height:", lt_dim['h'])
print("Evac camera width:", evac_dim['w'], "Camera height:", evac_dim['h'])

while True:
    if lt_stream.stopped is True or evac_stream.stopped is True:
        break
    
    lt_frame = lt_stream.read()
    cv2.imshow('line track frame', lt_frame)

    evac_frame = evac_stream.read()
    cv2.imshow('evac frame', evac_frame)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

lt_stream.stop()
evac_stream.stop()
cv2.destroyAllWindows()