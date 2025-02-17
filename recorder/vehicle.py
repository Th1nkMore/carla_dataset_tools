#!/usr/bin/python3
import copy
import csv
import os
import carla
import math

from recorder.actor import Actor
from recorder.agents.navigation.behavior_agent import BasicAgent
from recorder.agents.navigation.behavior_agent import BehaviorAgent


class OtherVehicle(Actor):
    def __init__(self,
                 uid,
                 name: str,
                 base_save_dir: str,
                 carla_actor: carla.Vehicle):
        super().__init__(uid=uid, name=name, parent=None,carla_actor=carla_actor)
        self.vehicle_type = copy.deepcopy(carla_actor.type_id)
        self.save_dir = '{}/{}_{}'.format(base_save_dir, self.vehicle_type, self.get_uid())
        self.first_tick = True
        # For vehicle control
        self.auto_pilot = True
        self.vehicle_agent = None
        self.carla_actor = carla_actor

    def get_type_id(self):
        return 'others.other_vehicle'

    def save_to_disk(self, frame_id, timestamp, debug=False):
        # Other vehicle not saving data
        return

    def get_save_dir(self):
        return self.save_dir
    

    def get_carla_bbox(self):
        return self.carla_actor.bounding_box

    def get_carla_transform(self):
        return self.carla_actor.get_transform()


    def control_step(self):
        # TODO: Migration with agents.behavior_agent
        self.carla_actor.set_autopilot()
        # if not self.auto_pilot:
        #     self.carla_actor.set_autopilot()
        #     self.auto_pilot = True
        # else:
        #     return

    def get_carla_actor(self):
        return self.carla_actor


class Vehicle(Actor):
    def __init__(self,
                 uid,
                 name: str,
                 base_save_dir: str,
                 carla_world: carla.World,
                 carla_actor: carla.Vehicle):
        super().__init__(uid=uid, name=name, parent=None, carla_actor=carla_actor)
        self.carla_world = carla_world
        self.vehicle_type = copy.deepcopy(carla_actor.type_id)
        self.save_dir = '{}/{}'.format(base_save_dir, self.name)
        self.first_tick = True
        # For vehicle control
        self.use_auto_pilot = True
        # self.vehicle_agent = BasicAgent(self.carla_actor)
        # self.vehicle_agent = BehaviorAgent(self.carla_actor)
        self.carla_actor= carla_actor

    def get_save_dir(self):
        return self.save_dir

    def get_carla_bbox(self):
        return self.carla_actor.bounding_box

    def get_carla_transform(self):
        return self.carla_actor.get_transform()

    def get_control(self):
        """
        Get vehicle control command.
        :return: vehicle control command.
        """
        return self.carla_actor.get_control()
    
    def get_carla_actor(self):
        return self.carla_actor

    @staticmethod
    def vehicle_control_to_dict(vehicle_control: carla.VehicleControl) -> dict:
        return {'throttle': vehicle_control.throttle,
                'brake': vehicle_control.brake,
                'steer': vehicle_control.steer,
                'reverse': vehicle_control.reverse,
                'gear': vehicle_control.gear}

    def complexity(self, npc, dist):
        t = npc.get_velocity()
        velocity = 3.6 * math.sqrt(t.x * t.x
                               + t.y * t.y
                               + t.z * t.z)
        return velocity + min(50, 1/dist**2)

    def save_to_disk(self, frame_id, timestamp, debug=False):
        os.makedirs(self.save_dir, exist_ok=True)
        fieldnames = ['frame',
                      'timestamp',
                      'x', 'y', 'z',
                      'roll', 'pitch', 'yaw',
                      'speed',
                      'vx', 'vy', 'vz',
                      'ax', 'ay', 'az',
                      'throttle', 'brake',
                      'steer', 'reverse', 'gear','count']

        if self.first_tick:
            self.save_vehicle_info()
            with open('{}/vehicle_status.csv'.format(self.save_dir), 'w', encoding='utf-8') as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                if self.first_tick:
                    writer.writeheader()
                    self.first_tick = False

        # Save vehicle status to csv file
        # frame_id x, y, z, roll, pitch, yaw, speed, acceleration
        count=0
        try:
            for npc in self.carla_world.get_actors().filter('*vehicle*'):
                if npc.id != self.carla_actor.id:
                    dist = npc.get_transform().location.distance(self.carla_actor.get_transform().location)
                    # Filter for the vehicles within 50m
                    if dist < 50:
                        count +=  self.complexity(npc, dist)
        except Exception as e:
            print(f"Caught an exception: {e}\n")
        with open('{}/vehicle_status.csv'.format(self.save_dir), 'a', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            csv_line = {'frame': frame_id,
                        'timestamp': timestamp,
                        'speed': self.get_speed(),
                        'count': count}
            csv_line.update(self.get_acceleration().to_dict(prefix='a'))
            csv_line.update(self.get_velocity().to_dict(prefix='v'))
            csv_line.update(self.get_transform().to_dict())
            csv_line.update(self.vehicle_control_to_dict(self.get_control()))
            writer.writerow(csv_line)

        if debug:
            print("\tVehicle status recorded: uid={} name={}".format(self.uid, self.name))

    def save_vehicle_info(self):
        # TODO: Save vehicle physics info here
        pass

    def control_step(self):
        # TODO: Migration with agents.behavior_agent
        if self.use_auto_pilot:
            self.carla_actor.set_autopilot()
        else:
            self.carla_actor.apply_control(self.vehicle_agent.run_step())

        # if not self.auto_pilot:
        #     self.carla_actor.set_autopilot()
        #     self.auto_pilot = True
        # else:
        #     return
