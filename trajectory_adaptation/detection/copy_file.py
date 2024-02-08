import os
import shutil
from tqdm import tqdm
#modify the dataset moving some files


# Define paths
source_folder = '/home/alessandro/Dataset/trajectory/robcamhalf'
destination_folder = '/home/alessandro/Dataset/trajectory/centroid_and_state'
a = os.listdir(source_folder)
progress_bar = tqdm(total=len(a))
# Iterate through each subfolder in the source folder
for subfolder_name in os.listdir(source_folder):
    subfolder_path = os.path.join(source_folder, subfolder_name)
    
    
    # Check if the path is a directory
    if os.path.isdir(subfolder_path):
        # Create corresponding subfolder in the destination folder
        destination_subfolder_path = os.path.join(destination_folder, subfolder_name)
        os.makedirs(destination_subfolder_path, exist_ok=True)
        
        # Iterate through files in the subfolder
        files = os.listdir(subfolder_path)
        # Copy only the first two files to the destination subfolder
        for file_name in files:
            if file_name == 'bottom_camera_centroids.csv' or file_name == 'robot_state.csv':
                source_file_path = os.path.join(subfolder_path, file_name)
                destination_file_path = os.path.join(destination_subfolder_path, file_name)
                shutil.copyfile(source_file_path, destination_file_path)
    progress_bar.update(1)
progress_bar.close()