# merge folder shapes_images and robo_images into one folder merged_images
import os
import shutil

# Define source folders and destination folder
source_folders = ["shapes_images", "robo_images"]
destination_folder = "merged_images"
# Create destination folder if it doesn't exist
os.makedirs(destination_folder, exist_ok=True)
# Loop through each source folder
for folder in source_folders:
    # Loop through each file in the source folder
    for filename in os.listdir(folder):
        source_path = os.path.join(folder, filename)
        destination_path = os.path.join(destination_folder, filename)
        # Move the file to the destination folder
        shutil.move(source_path, destination_path)
        print(f"Moved: {source_path} to {destination_path}")

print("All images have been merged into merged_images folder.")
