#!/usr/bin/python3
import glob
import os
import sys
from pathlib import Path
from multiprocessing import Pool as ProcessPool
from sewar.full_ref import mse, rmse, psnr, uqi, ssim, ergas, scc, rase, sam, msssim, vifp

import pandas as pd

sys.path.append(Path(__file__).resolve().parent.parent.as_posix())  # repo path
from param import RAW_DATA_PATH, DATASET_PATH
from label_tools.yolov5.yolov5_helper import *

def gather_dataset(record_name: str):
    yolo_dataset_df = pd.DataFrame()
    image_path_list = sorted(glob.glob(f"{DATASET_PATH}/{record_name}/yolo_dataset/images/train/*.jpg"))
    labels_path_list = sorted(glob.glob(f"{DATASET_PATH}/{record_name}/yolo_dataset/labels/train/*.txt"))
    yolo_dataset_df['image_path'] = image_path_list
    yolo_dataset_df['label_path'] = labels_path_list
    return yolo_dataset_df

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--record', '-r',
        required=True,
        help='Rawdata Record ID. e.g. record_2022_0113_1337'
    )
    args = argparser.parse_args()
    rawdata_df = gather_dataset(args.record)
    for dx, image_path in enumerate(rawdata_df['image_path']):
        t = image_path.split(".jpg")[0]
        k = rawdata_df['label_path'][dx].split(".txt")[0]
        os.rename(image_path, t+"X.jpg")
        os.rename(rawdata_df['label_path'][dx], k+"X.txt")

            
if __name__ == '__main__':
    main()