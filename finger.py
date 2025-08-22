import cv2
import mediapipe as mp

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles

TIP_IDS = [4, 8, 12, 16, 20]
PIP_IDS = [3, 6, 10, 14, 18]

def count_fingers(hand_landmarks, handedness_label, img_h, img_w):
    lm = []
    for i in range(21):
        x = int(hand_landmarks.landmark[i].x * img_w)
        y = int(hand_landmarks.landmark[i].y * img_h)
        lm.append((x, y))

    fingers = 0
    thumb_tip, thumb_ip = lm[4], lm[3]
    if handedness_label == "Right":
        if thumb_tip[0] > thumb_ip[0]:
            fingers += 1
    else:
        if thumb_tip[0] < thumb_ip[0]:
            fingers += 1

    for tip_id, pip_id in zip(TIP_IDS[1:], PIP_IDS[1:]):
        if lm[tip_id][1] < lm[pip_id][1]:
            fingers += 1
    return fingers

def main():
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    with mp_hands.Hands(
        model_complexity=0,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    ) as hands:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("❌ Frame capture failed")
                break

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb)

            h, w = frame.shape[:2]
            total_count = 0

            if results.multi_hand_landmarks and results.multi_handedness:
                for hand_landmarks, handedness in zip(
                    results.multi_hand_landmarks, results.multi_handedness
                ):
                    label = handedness.classification[0].label
                    mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_styles.get_default_hand_landmarks_style(),
                        mp_styles.get_default_hand_connections_style()
                    )
                    cnt = count_fingers(hand_landmarks, label, h, w)
                    total_count += cnt
                    wx, wy = int(hand_landmarks.landmark[0].x * w), int(hand_landmarks.landmark[0].y * h)
                    cv2.putText(frame, f'{label}: {cnt}', (wx-40, wy-20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

            cv2.rectangle(frame, (10, 10), (260, 70), (0, 0, 0), -1)
            cv2.putText(frame, f'Total Fingers: {total_count}', (20, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2)

            cv2.imshow("Finger Counter (Windows)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
