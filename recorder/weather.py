#!/usr/bin/python3
import carla
from recorder.actor import PseudoActor
import os
import sys
import math


def clamp(value, minimum=0.0, maximum=100.0):
    return max(minimum, min(value, maximum))


class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0

    def tick(self, delta_seconds):
        self._t += 0.008 * delta_seconds
        self._t %= 2.0 * math.pi
        self.azimuth += 0.25 * delta_seconds
        self.azimuth %= 360.0
        self.altitude = (70 * math.sin(self._t)) - 20

    def __str__(self):
        return 'Sun(alt: %.2f, azm: %.2f)' % (self.altitude, self.azimuth)


class Storm(object):
    def __init__(self, precipitation):
        self._t = precipitation if precipitation > 0.0 else -50.0
        self._increasing = True
        self.clouds = 0.0
        self.rain = 0.0
        self.wetness = 0.0
        self.puddles = 0.0
        self.wind = 0.0
        self.fog = 0.0

    def tick(self, delta_seconds):
        delta = (1.3 if self._increasing else -1.3) * delta_seconds
        self._t = clamp(delta + self._t, -250.0, 100.0)
        self.clouds = clamp(self._t + 40.0, 0.0, 90.0)
        self.rain = clamp(self._t, 0.0, 80.0)
        delay = -10.0 if self._increasing else 90.0
        self.puddles = clamp(self._t + delay, 0.0, 85.0)
        self.wetness = clamp(self._t * 5, 0.0, 100.0)
        self.wind = 5.0 if self.clouds <= 20 else 90 if self.clouds >= 70 else 40
        self.fog = clamp(self._t - 10, 0.0, 30.0)
        if self._t == -250.0:
            self._increasing = True
        if self._t == 100.0:
            self._increasing = False

    def __str__(self):
        return 'Storm(clouds=%d%%, rain=%d%%, wind=%d%%)' % (self.clouds, self.rain, self.wind)


class WeatherActor(PseudoActor):
    def __init__(self,
                 uid,
                 base_save_dir,
                 world,
                 weather):
        super().__init__(uid=uid, name=self.get_type_id(), parent=None)
        self.weather = weather
        self.save_dir = "{}/{}_{}".format(base_save_dir, self.get_type_id(), uid)
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        self._storm = Storm(weather.precipitation)
        self.world = world
    def get_type_id(self):
        return 'weather'

    def tick(self, delta_seconds):
        self._sun.tick(delta_seconds)
        self._storm.tick(delta_seconds)
        self.weather.cloudiness = self._storm.clouds
        self.weather.precipitation = self._storm.rain
        self.weather.precipitation_deposits = self._storm.puddles
        self.weather.wind_intensity = self._storm.wind
        self.weather.fog_density = self._storm.fog
        self.weather.wetness = self._storm.wetness
        self.weather.sun_azimuth_angle = self._sun.azimuth
        self.weather.sun_altitude_angle = self._sun.altitude
        
    def save_to_disk(self, frame_id, timestamp, debug=True):
        os.makedirs(self.save_dir, exist_ok=True)
        speed_factor = 10 # default 1
        # update_freq = 0.1 / speed_factor
        elapsed_time = 0.2
        try:
          self.tick(speed_factor * elapsed_time)
          self.world.set_weather(self.weather)
          # print('\rWeather:' + str(self) + 12 * ' ')
          with open("{}/{:0>10d}.txt".format(self.save_dir,
                                                        frame_id),'w') as f:
                                    f.write('Weather:' + str(self) + 12 * ' ' +'\n')
        except Exception as e:
          print("Weather:"+e)
        if debug:
            print('\r' + str(self) + 12 * ' ')
            # sys.stdout.flush()
        
        return super().save_to_disk(frame_id, timestamp, debug = False)
    def get_save_dir(self):
        return self.save_dir
    def __str__(self):
        return '%s %s' % (self._sun, self._storm)

    
