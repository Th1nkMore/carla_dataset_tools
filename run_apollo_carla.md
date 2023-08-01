# How to use apollo-carla co-simulator?

```shell
# run apollo
docker start apollo_dev_ghosnp
docker exec -ti apollo_dev_ghosnp bash
./scripts/bootstrap.sh
```

```shell
# run bridge
cd carla/carla_apollo_bridge
cd docker/
./build_docker.sh
./run_docker.sh
docker exec -ti carla_cyber_0.9.14 bash

./apollo.sh build_cyber opt

# terminal A
cd cyber/carla_brigde/
python carla_cyber_bridge/bridge.py

# terminal B
cd cyber/carla_bridge/
python carla_swapn_objects/carla_spawn_objects.py
```

```shell
# run carla
conda activate carla38
cd carla/carla_0.9.14
./CarlaUE4.sh
```

```shell
# start
https://localhost:8888
```
