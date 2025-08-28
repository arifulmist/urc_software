import cv2

cap = cv2.VideoCapture(1)  # তোমার webcam index
for w, h in [(640,480), (1280,720), (1920,1080), (2560,1440)]:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    ret, frame = cap.read()
    if ret:
        print(f"✅ Supported: {w}x{h}")
    else:
        print(f"❌ Not supported: {w}x{h}")
cap.release()
