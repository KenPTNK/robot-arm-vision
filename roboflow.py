import os

from roboflow import Roboflow

# Read the API key from the environment: set ROBOFLOW_API_KEY before running
rf = Roboflow(api_key=os.environ["ROBOFLOW_API_KEY"])

# Retrieve your current workspace and project name
print(rf.workspace())

# Specify the project for upload
# let's you have a project at https://app.roboflow.com/my-workspace/my-project
workspaceId = 'my-workspace'
projectId = 'robot-camera-v2'
project = rf.workspace(workspaceId).project(projectId)

# Upload the image to your project
project.upload("UPLOAD_IMAGE.jpg")

"""
Optional Parameters:
- num_retry_uploads: Number of retries for uploading the image in case of failure.
- batch_name: Upload the image to a specific batch.
- split: Upload the image to a specific split.
- tag: Store metadata as a tag on the image.
- sequence_number: [Optional] If you want to keep the order of your images in the dataset, pass sequence_number and sequence_size..
- sequence_size: [Optional] The total number of images in the sequence. Defaults to 100,000 if not set.
"""

project.upload(
    image_path="UPLOAD_IMAGE.jpg",
    batch_name="YOUR_BATCH_NAME",
    split="train",
    num_retry_uploads=3,
    tag_names=["YOUR_TAG_NAME"],
    sequence_number=99,
    sequence_size=100
)