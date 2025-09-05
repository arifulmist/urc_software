import cv2
import numpy as np
focal_len=800
known_marker_size=5
arucodic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters=cv2.aruco.DetectorParameters()
cap=cv2.VideoCapture(1)
while True:
    ret,frame=cap.read()
    if not ret:
        break

    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    corners,ids,rej=cv2.aruco.detectMarkers(gray,arucodic,parameters=parameters)
    if ids is not None:
         cv2.aruco.drawDetectedMarkers(frame,corners,ids)
         for corner,marker_id in zip(corners,ids):
            #marker draw
           
            (topleft,topright,bottomright,bottomleft)=corner[0]
            topleft,topright,bottomright,bottomleft=map(lambda x: (int (x[0]),int (x[1])),[topleft,topright,bottomright,bottomleft])
            cx=int((topleft[0]+bottomright[0])/2.0)
            cy=int((topleft[1]+bottomright[1])/2.0)
            marker_width_pixels = np.linalg.norm(np.array(topright) - np.array(topleft))
            distance = (known_marker_size* focal_len) / marker_width_pixels
            cv2.circle(frame,(cx,cy),5,(0,0,255),-1)
            cv2.putText(frame,f"Id:{marker_id[0]} Dist:{distance:.2f}cm",
                        (topleft[0],topleft[1]-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
            

    cv2.imshow("Aruco Detection",frame)
    if cv2.waitKey(1) & 0xFF==ord('q'):
        break
cap.release()
cv2.destroyAllWindows()