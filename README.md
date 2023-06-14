# Carla Dataset Tools (Active Collecting Integrated)

Data collection tools with active data acquisition strategy implemented for CARLA Simulator.

## Installation

```
pip3 install -r requirements.txt

# Write following env into your bashrc or zshrc
export CARLA_ROOT=[PATH_TO_YOUR_CARLA]
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/[YOUR_CARLA_EGG_NAME]:$CARLA_ROOT/PythonAPI/carla/

```

## Usage

### Data Recording

Execute the command in the root directory:

```
python3 data_recorder.py
```

### Dataset Preparing

#### YOLO

Execute the command in the root directory:

```
python format_helper.py -s raw_data/record_2022_0119_1303
```

## Data

Data can be collected by just executing `python3 data_recorder.py`. The configuration can be changed by modifying the files in the folder `config`.

| Dataset Name | Total Frames | Map    |
| ------------ | ------------ | ------ |
| D1           | 900          | Town02 |
| D1-S         | 579          | Town02 |
| D2           | 900          | Town02 |
| V            | 375          | Town03 |

## Result

| Dataset Name | Vehicle | Training Time | mAP   |
| ------------ | ------- | ------------- | ----- |
| D1           | 996     | 14min40s      | 0.508 |
| D1-S         | 996     | 11min32s      | 0.492 |
| D2           | 1835    | 16min07s      | 0.647 |

## Contributing

Thank you for your interest in contributing to this project! Contributions are highly appreciated and help improve the project for everyone. If you have any questions or need further assistance, please feel free to open an issue.

## Acknowledgments

- [CARLA Simulator](https://carla.org/)
- [CARLA Ros Bridge](https://github.com/carla-simulator/ros-bridge)
- [CARLA_INVS](https://github.com/zijianzhang/CARLA_INVS)

## Citation

```

```

## Future Work

To validate the correctness of the strategy, we'd better try multiple algorithms:

- [x] YOLO
- [ ] CenterPoint
