import cv2
import numpy as np
focal_len=800
known_marker_size=5.0
mn_spd=1000
mx_spd=2500
basespd=1500
kp=500
arucodic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters=cv2.aruco.DetectorParameters()
cap=cv2.VideoCapture(1)
while True:
    ret,frame=cap.read()
    if not ret:
        break
    frame_h,frame_w=frame.shape[:2]
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    corners,ids,_=cv2.aruco.detectMarkers(gray,arucodic,parameters=parameters)
    if ids is not None:
         cv2.aruco.drawDetectedMarkers(frame,corners,ids)
         for corner,marker_id in zip(corners,ids):
            
            (topleft,topright,bottomright,bottomleft)=corner[0]
            topleft,topright,bottomright,bottomleft=map(lambda x: (int (x[0]),int (x[1])),[topleft,topright,bottomright,bottomleft])
            cx=int((topleft[0]+bottomright[0])/2.0)
            cy=int((topleft[1]+bottomright[1])/2.0)
            marker_width_pixels = np.linalg.norm(np.array(topright) - np.array(topleft))
            distance = (known_marker_size* focal_len) / marker_width_pixels
            err=cx-(frame_w//2)
            norm=err/(frame_w//2)
            turn=kp*norm
            leftmot=basespd-turn
            rightmot=basespd+turn
            leftmot=max(mn_spd,min(mx_spd,leftmot))
            rightmot=max(mn_spd,min(mx_spd,rightmot))

            cv2.circle(frame,(cx,cy),5,(0,0,255),-1)
            cv2.putText(frame,f"Id:{marker_id[0]}",(topleft[0],topleft[1]-30),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,0),2)
            cv2.putText(frame,f"Dist:{distance:.1f}cm",(topleft[0],topleft[1]-10),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
            cv2.putText(frame, f"L:{int(leftmot)} R:{int(rightmot)}",
                        (10, frame_h - 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2)
            print(f"[Marker {marker_id[0]}] Dist:{distance:.1f}cm | Left:{int(leftmot)} Right:{int(rightmot)}")

    cv2.imshow("Aruco follower",frame)
    if cv2.waitKey(1) & 0xFF==ord('q'):
        break
cap.release()
cv2.destroyAllWindows()