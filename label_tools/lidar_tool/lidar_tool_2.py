import open3d as o3d
import os
import sys
import numpy as np
import argparse
import math
import carla

class labeler():
    usable_labels = {12.,14.,15.,16.,19.}
    label_dict = {12.:'Pedestrian',14.:'Car',15.:'Truck',16.:'Bus',19.:'Bicycle'}

    def __init__(self,world: carla.World, debug_helper: carla.DebugHelper, vehicle: carla.Actor):
        self.world = world
        self.vehicle = vehicle
        # Set up the set of bounding boxes from the level
        # We filter for traffic lights and traffic signs
        self.bounding_box_set = self.world.get_level_bbs(carla.CityObjectLabel.TrafficLight)
        self.bounding_box_set.extend(self.world.get_level_bbs(carla.CityObjectLabel.TrafficSigns))

        # Remember the edge pairs
        self.edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

    
    def get_near_boxes(self):
        for bb in self.bounding_box_set:
            # only label the near vehicle
            if bb.location.distance(self.vehicle.get_transform().location) < 50:
                forward_vec = self.vehicle.get_transform().location
                ray = bb.location - self.vehicle.get_transform().location

                # if forward_vec.dot(ray) > 1:


    


class strategy():

    def active_voxel_entroy(self,origin_entropy, pcd):
        import util
        voxel = util.Voxel()
        now_entropy = voxel.get_entropy_score(pcd)
        return abs((now_entropy - origin_entropy) / origin_entropy) > 0.2, now_entropy




