import cv2
import os

# Create folder if not exists
save_path = "robo_images"
os.makedirs(save_path, exist_ok=True)

# Open webcam (0 = default camera)
cap = cv2.VideoCapture(1)

count = 0

print("Press 's' to save image | Press 'q' to quit")

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame")
        break

    # Show camera
    cv2.imshow("Camera", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):  # press 's' to save
        filename = os.path.join(save_path, f"img_{count}.jpg")
        cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 100])
        print(f"Saved: {filename}")
        count += 1

    elif key == ord('q'):  # press 'q' to quit
        break

cap.release()
cv2.destroyAllWindows()