import pickle
from utils.label_types import *
def test():
    with open('/home/ubuntu/Carla/carla_dataset_tools/raw_data/record_2023_0224_1257/others.world_0/0000000008.pkl', 'rb') as f:
        data = pickle.load(f)
        print(data)
        print(type(data))