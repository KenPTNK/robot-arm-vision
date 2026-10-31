import cv2
import os

# Create folder if it doesn't exist
save_dir = "calib_images3"
os.makedirs(save_dir, exist_ok=True)

# Start webcam (0 = default camera)
cap = cv2.VideoCapture(1)

img_count = 0

print("Press 's' to save image, 'q' to quit.")

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame")
        break

    # Show the video feed
    cv2.imshow("Camera", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        # Save image
        img_name = os.path.join(save_dir, f"image_{img_count}.png")
        cv2.imwrite(img_name, frame)
        print(f"Saved {img_name}")
        img_count += 1

    elif key == ord('q'):
        # Quit
        print("Exiting...")
        break

# Release resources
cap.release()
cv2.destroyAllWindows()