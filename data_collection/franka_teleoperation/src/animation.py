import cv2
import numpy as np

# Load your NumPy array from the .npy file
data = np.load('/home/alessandro/Dataset/localization/Pushing_Single_Strawberry_training/second_collection/data_sample_2023-12-04-09-49-17/camera_finger.npy')

# Determine video dimensions based on the shape of the array
q, height, width , a = data.shape

# Choose a codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can choose other codecs like 'XVID' or 'MJPG'
out = cv2.VideoWriter('output_video.mp4', fourcc, 20.0, (width, height))

# Iterate through each frame in the array and write it to the video
for frame in data:
    # Ensure the frame data is in the correct format (e.g., uint8)
    frame = frame.astype(np.uint8)
    
    # Write the frame to the video file
    out.write(frame)

# Release the VideoWriter object
out.release()

print("Video created successfully.")