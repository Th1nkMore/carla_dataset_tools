import os
import shutil
import argparse

def split_files(source_dir, png_dir, txt_dir):
    os.makedirs(png_dir, exist_ok=True)
    os.makedirs(txt_dir, exist_ok=True)
    for filename in os.listdir(source_dir):
        if filename.endswith('.png'):
            shutil.copy(os.path.join(source_dir, filename), os.path.join(png_dir, source_dir.split('/')[2]+"_"+filename))
        elif filename.endswith('.txt'):
            shutil.copy(os.path.join(source_dir, filename), os.path.join(txt_dir, source_dir.split('/')[2]+"_"+filename))

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Format Helper')
    parser.add_argument('--source', '-s', type=str, help='source directory')
    args = parser.parse_args()
    source = args.source
    data = []
    for entry in os.scandir(source):
        if entry.is_dir() and entry.name.startswith("vehicle"):
            data.append(source+"/"+entry.name+"/image_2")
    for source_dir in data:
        split_files(source_dir, 'dataset/test/images', 'dataset/test/labels')    

    # print(data[0].split('/'))