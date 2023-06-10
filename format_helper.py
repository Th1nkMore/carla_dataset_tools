import os
import shutil

def split_files(source_dir, png_dir, txt_dir):
    # Create destination directories if they don't exist
    os.makedirs(png_dir, exist_ok=True)
    os.makedirs(txt_dir, exist_ok=True)
    # Iterate over files in the source directory
    for filename in os.listdir(source_dir):
        if filename.endswith('.png'):
            # Move PNG files to the PNG directory
            shutil.copy(os.path.join(source_dir, filename), os.path.join(png_dir, filename))
        elif filename.endswith('.txt'):
            # Move TXT files to the TXT directory
            shutil.copy(os.path.join(source_dir, filename), os.path.join(txt_dir, filename))

# Specify the source directory and the destination directories for PNG and TXT files
source = 'raw_data/record_2023_0525_1743/vehicle.tesla.model3.master/image_2'
png_directory = 'dataset/test/images'
txt_directory = 'dataset/test/labels'

# Call the split_files function
split_files(source, png_directory, txt_directory)
