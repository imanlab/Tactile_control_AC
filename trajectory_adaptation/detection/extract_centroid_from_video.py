import numpy as np
import cv2
import os
import pandas as pd
from tqdm import tqdm




def format_for_save(cx_list, cy_list):
    CX=np.asarray(cx_list)
    CY=np.asarray(cy_list)
    cent = [CX,CY]
    centroids = np.transpose(np.asarray(cent))
    df = pd.DataFrame(centroids)

    return df

def save_data(df,path):
    col_names = ["C_x", "C_y"]
    df.to_csv(path, header=col_names, index = False)



# Define the lower and upper bounds for the red color in HSV color space
def process_video(video_array):
    lower_red = np.array([0,100,100])
    upper_red = np.array([10,255,255])
    i = 0
    roi_coordinates = (0, 0, 1280, 560)
    cx_list = []
    cy_list = []
    # Process each frame
    for frame in video_array:
        cropped_frame = frame[roi_coordinates[1]:roi_coordinates[1]+roi_coordinates[3],
                            roi_coordinates[0]:roi_coordinates[0]+roi_coordinates[2]]
        # Convert frame to HSV color space
        hsv_frame = cv2.cvtColor(cropped_frame, cv2.COLOR_RGB2HSV)

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv_frame, lower_red, upper_red)

        # Find contours in the masked image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea, default = None)
        
    
        if largest_contour is not None:
            i=i+1
            leftmost = tuple(largest_contour[largest_contour[:,:,0].argmin()][0])
            rightmost = tuple(largest_contour[largest_contour[:,:,0].argmax()][0])
            topmost = tuple(largest_contour[largest_contour[:,:,1].argmin()][0])
            bottommost = tuple(largest_contour[largest_contour[:,:,1].argmax()][0])

            cx=int((leftmost[0]+rightmost[0])/2)
            cy=int((topmost[1]+bottommost[1])/2)
            
            cx_list.append(cx)
            cy_list.append(cy)

        
    centroids = format_for_save(cx_list,cy_list)
    return centroids
# Load the video
path = '/home/alessandro/Dataset/trajectory/robcamhalf'#data_sample_2023-12-17-17-31-06'
# video_array = np.load('/home/alessandro/Dataset/trajectory/robcamhalf/data_sample_2023-12-17-17-31-06/bottom_camera.npy') 
# print(centroids)
# save_data(centroids)
files = os.listdir(path)
progress_bar = tqdm(total=len(files))

for root, dirs, files in os.walk(path):
    for file in files:
        if file.endswith('.npy'):
            filepath = os.path.join(root, file)
            video_array = np.load(filepath)
            centroids = process_video(video_array)
            save_data(centroids, filepath.replace('.npy', '_centroids.csv'))
            progress_bar.update(1)
print("DONE!")
progress_bar.close()