import cv2
from ultralytics import YOLO

# LOAD MODEL
model = YOLO("old_remove_best.pt")

# OPEN CAMERA
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Could not open camera")
    exit()

print("Camera is running...")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # YOLO inference
    results = model(frame)
    boxes = results[0].boxes
    names = model.names

    # LOOP THROUGH DETECTIONS
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0]
        conf = float(box.conf[0])
        cls = int(box.cls[0])
        label = names[cls]

        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        # DRAW BOX
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        text = f"{label} {conf:.2f}"
        cv2.putText(frame, text, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # SHOW FRAME
    cv2.imshow("YOLO Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# RELEASE
cap.release()
cv2.destroyAllWindows()