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

# def gather_dataset(record_name: str, vehicle_name: str):
def gather_dataset(record_name: str):
    yolo_dataset_df = pd.DataFrame()
    # vehicle_dataset_path = f"{DATASET_PATH}/{record_name}/{vehicle_name}"
    # image_path_list = sorted(glob.glob(f"{vehicle_dataset_path}/yolo/yolo_dataset/images/train/*.jpg"))
    # labels_path_list = sorted(glob.glob(f"{vehicle_dataset_path}/yolo/yolo_dataset/labels/train/*.txt"))
    image_path_list = sorted(glob.glob(f"{DATASET_PATH}/{record_name}/yolo_dataset/images/train/*.jpg"))
    labels_path_list = sorted(glob.glob(f"{DATASET_PATH}/{record_name}/yolo_dataset/labels/train/*.txt"))
    yolo_dataset_df['image_path'] = image_path_list
    yolo_dataset_df['label_path'] = labels_path_list
    # yolo_dataset_df['vehicle_name'] = vehicle_name
    return yolo_dataset_df

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--record', '-r',
        required=True,
        help='Rawdata Record ID. e.g. record_2022_0113_1337'
    )
    args = argparser.parse_args()
    # vehicle_name_list = [os.path.basename(x) for x in
    #                          glob.glob('{}/{}/vehicle.*'.format(DATASET_PATH, args.record))]
    # for vehicle_name in vehicle_name_list:
    #     rawdata_df = gather_dataset(args.record,
    #                                 vehicle_name)
    #     print(vehicle_name)
    #     cur_img = cv2.imread(rawdata_df['image_path'][0], cv2.IMREAD_UNCHANGED)
    #     for dx, path in enumerate(rawdata_df['image_path']):
    #         if dx == 0 or dx == len(rawdata_df['image_path']):
    #             continue
    #         image = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    #         if uqi(cur_img, image)<0.95:
    #             cur_img = image
    #         else:
    #             print(f"remove {path}")
    #             os.remove(path)
    #             os.remove(rawdata_df['label_path'][dx])
    rawdata_df = gather_dataset(args.record)
    cur_img = cv2.imread(rawdata_df['image_path'][0], cv2.IMREAD_UNCHANGED)
    print(len(rawdata_df['image_path']))
        

            
if __name__ == '__main__':
    main()