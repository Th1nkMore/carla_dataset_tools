# How to visulaize result?

1. Using L-Shape fitting Algorithm **(Semantic Data)**
   
   ```shell
   cd label_tools/lidar_tool
   python lidar_tool.py -d <.bin semantic point cloud file>
   ```

2. Using 3D-model transform **(Origin Data)**
   
   ```shell
   cd label_tools/lidar_tool
   python vis_origin.py -d <dir number xxxx_xxxx> -s <.bin origin point cloud file>
   ```
   
   

# Source code getting transform

```shell
# See recorder/lidar.py, recorder/actor_factory.py
```


