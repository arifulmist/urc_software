import cv2

cap = cv2.VideoCapture(1)  # 0 = default webcam

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Cannot access webcam")
        break

    cv2.imshow("Webcam Test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
