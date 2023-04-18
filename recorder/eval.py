import pickle
from utils.label_types import *
def test():
    cnt = 0
    with open('/home/ubuntu/Carla/carla_dataset_tools/raw_data/record_2023_0414_1025/others.world_0/0000000008.pkl', 'rb') as f:
        data = pickle.load(f)
        for Object in data:
            print(Object)
            # print(f"id:{Object.carla_id}")
            # print(f"label type:{Object.label_type}")
        #     if Object.label_type== 'vehicle' and int(Object.carla_id)>200:
        #         cnt += 1
        # print(f"vehicle number: {cnt}")