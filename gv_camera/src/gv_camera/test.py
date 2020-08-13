import numpy as np
import cv2

def rescale_frame(Frame,percent=75):
    width = int(Frame.shape[1]*percent/100)
    height= int(Frame.shape[0]*percent/100)
    dim = (width,height)
    return cv2.resize(Frame,dim,interpolation=cv2.INTER_AREA)

cap = cv2.VideoCapture("rtsp://192.168.33.181:8554//CH001.sdp")

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (1280,1200))

while(cap.isOpened()):
    # print (cap.get(3), cap.get(4))
    ret, frame = cap.read()
    if ret==True:
        frame = cv2.flip(frame,0)

        # write the flipped frame
        # out.write(frame)
        frame=rescale_frame(frame,percent=75)
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()