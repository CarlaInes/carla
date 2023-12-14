#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementation.
It must not be modified and is for reference only!
"""

from __future__ import print_function
from cmath import sin, sqrt
from itertools import product
import logging
import math
import random
import sys
import time
import carla
import pandas as pd
import py_trees

from srunner.autoagents.agent_wrapper import AgentWrapper
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.result_writer import ResultOutputProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog

from srunner.scenariomanager.actorcontrols.visualizer import Visualizer


class ScenarioManager(object):

    """
    Basic scenario manager class. This class holds all functionality
    required to start, and analyze a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. Trigger a result evaluation with manager.analyze_scenario()
    5. If needed, cleanup with manager.stop_scenario()
    """

    def __init__(self, debug_mode=False, sync_mode=False, timeout=2.0, runnerTool_params = None):
        """
        Setups up the parameters, which will be filled at load_scenario()

        """
        self.scenario = None
        self.scenario_tree = None
        self.scenario_class = None
        self.ego_vehicles = None
        self.other_actors = None

        self._debug_mode = debug_mode
        self._agent = None
        self._sync_mode = sync_mode
        self._watchdog = None
        self._timeout = timeout

        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None

        self.runnerTool_cam = runnerTool_params["camera"]
        self.runnerTool_speed = runnerTool_params["speed"]

        self._visualizer = None


    def _reset(self):
        """
        Reset all parameters
        """
        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        GameTime.restart()

    def cleanup(self):
        """
        This function triggers a proper termination of a scenario
        """

        if self._watchdog is not None:
            self._watchdog.stop()
            self._watchdog = None

        if self.scenario is not None:
            self.scenario.terminate()

        if self._agent is not None:
            self._agent.cleanup()
            self._agent = None

        #runnerTool
        if self._visualizer is not None:
            self._visualizer.reset()

        CarlaDataProvider.cleanup()

    def load_scenario(self, scenario, agent=None):
        """
        Load a new scenario
        """
        self._reset()
        self._agent = AgentWrapper(agent) if agent else None
        if self._agent is not None:
            self._sync_mode = True
        self.scenario_class = scenario
        self.scenario = scenario.scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors

        #runnerTool set speed and camera
        world = CarlaDataProvider.get_world()

        if self.runnerTool_speed != 100:
            self.set_speed(world)

        if self.runnerTool_cam is not None and self.runnerTool_speed == 100: 
            self._visualizer = Visualizer(self.ego_vehicles[0])

        self.get_camera(world)

        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

        if self._agent is not None:
            self._agent.setup_sensors(self.ego_vehicles[0], self._debug_mode)
    
    ####################runnerTool:
    def set_speed(self, world):
        ''' [RUNNERTOOL] changes scenario display speed'''
        print("Setting Scenario Speed to %.2fX" %(self.runnerTool_speed/100))
        settings = world.get_settings()
        if self.runnerTool_speed == 100:
            settings.fixed_delta_seconds = None
        else:
            settings.fixed_delta_seconds = (1.0 / (140/(self.runnerTool_speed/100)))
        world.apply_settings(settings)

    def get_camera(self,world):
        ''' [RUNNERTOOL] creates initial spectator position above ego vehicle'''
        spectator = world.get_spectator()
        target = world.get_actor(self.ego_vehicles[0].id).get_location()
        transform = carla.Transform(carla.Location(x = target.x, y = target.y, z = 60), carla.Rotation(pitch=270, yaw=0, roll=0))
        spectator.set_transform(transform) 
        time.sleep(1)

    def reset_camera(self, actor, spectator):
        ''' [RUNNERTOOL] fixes spectator to ego vehicle either in bird or ego perspective. 
            Removed with runnerTool v1.01 due to problems in comination with runnerTool_speed settings.
            Instead visualizer is used if runnerTool_speed is 100 i.e. not changed from default
        '''

        raise NotImplementedError

        target = actor.get_location()
        if self.runnerTool_cam == "bird":
            transform = carla.Transform(carla.Location(x = target.x, y = target.y, z = 60), carla.Rotation(pitch=270, yaw=0, roll=0))
        elif self.runnerTool_cam == "ego":
            transform = carla.Transform(carla.Location(x = target.x, y = target.y, z = 3), actor.get_transform().rotation)
        else:
            print("ERROR could not resolve camera setting")

        spectator.set_transform(transform) 
    ####################
            
    ####################################################################################################################################
    # Functions to get information about the ego vehicle
    def get_speed(self,ego_vehicle):
        # Get the velocity of the ego vehicle
        velocity = ego_vehicle.get_velocity()
        speed = velocity.length()
        return speed #in m/s

    def get_acceleration(self,speed, previous_speed, current_time, previous_time):
        time_interval = (current_time - previous_time) / 1_000_000_000
        acceleration = (speed - previous_speed) / time_interval  # Change in speed during one time interval
        return acceleration

    def get_steering_angle(self,ego_vehicle):
        """
        steering_angle < 0: driving left
        steering_angle > 0: driving right
        """
        control = ego_vehicle.get_control()

        steering_angle = control.steer * 180 #angle in degree, control.steer in [-1.0, 1.0]
        #steering_angle = steering_angle * (math.pi / 360) #angle in radians
        return steering_angle

    def get_lateral_acceleration(self,ego_vehicle, current_time, previous_time, previous_yaw, speed):
        time_interval = (current_time - previous_time) / 1_000_000_000
        # Get the lateral rotation (yaw) of the ego vehicle
        current_yaw = ego_vehicle.get_transform().rotation.yaw
        delta_yaw = math.radians(current_yaw - previous_yaw)

        lateral_acceleration = (speed * delta_yaw) / time_interval

        return lateral_acceleration, current_yaw


    def get_dist_to_lane_center(self,ego_vehicle, world):
        ego_location = ego_vehicle.get_location()
        waypoint = world.get_map().get_waypoint(ego_location)
        lane_location = waypoint.transform.location
        dist_to_lane_center = math.sqrt((lane_location.x - ego_location.x) **2 + (lane_location.y - ego_location.y) ** 2)

        return dist_to_lane_center


    def get_speed_of_vehicle_ahead(self, ego_waypoint, world, max_distance=10):
        #camera_data = camera_sensor.listen()
        map = world.get_map()
        actor_locations = [(self.get_speed(actor), map.get_waypoint(actor.get_location()).transform.location) for actor in world.get_actors()]
        for i in range(1, max_distance + 1):
            next_waypoint = ego_waypoint.next(i)[0]
            for actor_speed, actor_location in actor_locations:
                if actor_location.distance(next_waypoint.transform.location) < 1:
                    return actor_speed
                
        return None
                

    def get_speed_of_vehicle_ahead_efficient(self, ego_waypoint, max_distance=10):
        #camera_data = camera_sensor.listen()
        actor_locations = [(self.get_speed(actor), map.get_waypoint(actor.get_location()).transform.location) for actor in world.get_actors()]

        for i, actor_speed, actor_location in product(range(1, max_distance + 1), *zip(*actor_locations)):
            next_waypoint = ego_waypoint.next(i)[0]
            distance = actor_location.distance(next_waypoint.transform.location)
            if distance < 1:
                return actor_speed

        # Return a default value if no valid speed is found
        return None

    # Function to get the curvature of the road at the ego vehicle's current location in degrees
    def get_curvature_at_location(self, vehicle_location, world):
        map = world.get_map()
        # Find the nearest waypoint corresponding to the vehicle's location
        waypoint = map.get_waypoint(vehicle_location)
        # Get the curvature at the waypoint
        curvature_coeff = self.get_curvature_coeff(waypoint, 1)

        return curvature_coeff


    def get_curvature_coeff(self,ego_waypoint, route_distance):
        previous_waypoint = ego_waypoint.previous(route_distance)[0]
        next_waypoint = ego_waypoint.next(route_distance)[0]
        _transform = next_waypoint.transform
        _location, _rotation  = _transform.location, _transform.rotation
        x1, y1 = _location.x, _location.y
        yaw1 = _rotation.yaw

        _transform = previous_waypoint.transform
        _location, _rotation  = _transform.location, _transform.rotation
        x2, y2 = _location.x, _location.y
        yaw2 = _rotation.yaw

        c = 2*sin(math.radians((yaw1-yaw2)/2)) / sqrt((x1-x2)**2 + (y1-y2)**2)
        return c
    ####################################################################################################################################
        


    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        print("ScenarioManager: Running scenario {}".format(self.scenario_tree.name))
        self.start_system_time = time.time()
        start_game_time = GameTime.get_time()

        self._watchdog = Watchdog(float(self._timeout))
        self._watchdog.start()
        self._running = True

        # Create an empty list to store the data as dictionaries
        data_list = []

        previous_speed = 0  # Store the previous speed value in m/s
        previous_yaw = 0  # Store the previous yaw angle value
        leading_vehicle = None  # Variable to store the leading vehicle reference
        previous_time = time.time_ns()

        world = CarlaDataProvider.get_world()          

        time_interval = 1.0 #for acceleration, in s
        ego_vehicle_bp = world.get_blueprint_library().find('vehicle.audi.a2')


        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            ego_transform = spawn_points[0]
            ego_vehicle = world.spawn_actor(ego_vehicle_bp,ego_transform)
            print('\nEgo is spawned')
        else: 
            logging.warning('Could not find any spawn points')
        print("initialization successful")





        #k = 0
        while self._running:
            timestamp = None
            world = CarlaDataProvider.get_world()          
            if world:
                #if self._visualizer is None and self.runnerTool_cam is not None:
                    #self._visualizer.render()
                    #if k % 5000 == 0:
                        #self.reset_camera(world.get_actor(self.ego_vehicles[0].id),world.get_spectator())

                snapshot = world.get_snapshot()
                print("start of data collection")
                #TODO: adding our (or at least my code) here
                 #returns the current speed of the ego vehicle in m/s
                speed = self.get_speed(ego_vehicle)
                current_time = time.time_ns()

                acceleration = self.get_acceleration(speed, previous_speed, current_time, previous_time)

                steering_angle = self.get_steering_angle(ego_vehicle) #TODO angle in degree or radians?

                lateral_acceleration, current_yaw = self.get_lateral_acceleration(ego_vehicle, current_time, previous_time, previous_yaw, speed)

                dist_to_lane_center = self.get_dist_to_lane_center(ego_vehicle, world) #in m #TODO needs to be tested with manual car

                ego_location = ego_vehicle.get_location()
                waypoint = world.get_map().get_waypoint(ego_location)
                speed_vehicle_ahead = self.get_speed_of_vehicle_ahead(waypoint, world) 

                # Get and print the curvature at the ego vehicle's location in degrees per meter
                curvature_degrees_per_meter = self.get_curvature_at_location(ego_vehicle.get_location(), world) 

                #print({'Speed (m/s)': speed, '\nAcceleration (m/s^2)': acceleration, '\nSteering Angle' : steering_angle, '\nLateral Acceleration' : lateral_acceleration})
                #print('\nCurvature at Ego Vehicle Location (Degrees/m): ' + str(curvature_degrees_per_meter))
            


                # Append the data as a dictionary to the list
                data_list.append({'Speed (m/s)\t\t': speed, 'Acceleration (m/s^2)\t\t': acceleration, 'Steering Angle\t\t' : steering_angle,\
                                'Lateral Acceleration (m/s^2)\t\t' : lateral_acceleration, 'Distance to lane center (m)\t\t' : dist_to_lane_center,\
                                    'Speed of vehicle ahead (m/s)\t\t' : speed_vehicle_ahead,\
                                    'Road Curvature at Ego Vehicle Location (Degrees/m)\t\t' : curvature_degrees_per_meter})
                
                print(speed)
                

                previous_speed = speed  # Update the previous speed value
                previous_time = current_time
                previous_yaw = current_yaw  # Update the previous yaw angle value
                print("end of data collection")

                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                self._tick_scenario(timestamp)
            #k+=1

        self.cleanup()

        # Convert the list of dictionaries to a DataFrame
        data_df = pd.DataFrame(data_list)

        #Save data from the dataframe in a csv file
        filename = 'test_data/' + time.strftime("%Y-%m-%d_%H-%M-%S") + '.csv'
        data_df.to_csv(filename, index=False)

        self.end_system_time = time.time()
        end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - \
            self.start_system_time
        self.scenario_duration_game = end_game_time - start_game_time

        if self.scenario_tree.status == py_trees.common.Status.FAILURE:
            print("ScenarioManager: Terminated due to failure")

    def _tick_scenario(self, timestamp):
        """
        Run next tick of scenario and the agent.
        If running synchornously, it also handles the ticking of the world.
        """

        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()


            if self._debug_mode:
                print("\n--------- Tick ---------\n")

            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            ##runnerTool
            if self._visualizer is not None:
                self._visualizer.render()
            ##
            if self._agent is not None:
                ego_action = self._agent()  # pylint: disable=not-callable

            if self._agent is not None:
                self.ego_vehicles[0].apply_control(ego_action)

            # Tick scenario
            self.scenario_tree.tick_once()

            if self._debug_mode:
                print("\n")
                py_trees.display.print_ascii_tree(self.scenario_tree, show_status=True)
                sys.stdout.flush()

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                self._running = False

        if self._sync_mode and self._running and self._watchdog.get_status():
            CarlaDataProvider.get_world().tick()

    def get_running_status(self):
        """
        returns:
           bool:  False if watchdog exception occured, True otherwise
        """
        return self._watchdog.get_status()

    def stop_scenario(self):
        """
        This function is used by the overall signal handler to terminate the scenario execution
        """
        self._running = False

    def analyze_scenario(self, stdout, filename, junit, json):
        """
        This function is intended to be called from outside and provide
        the final statistics about the scenario (human-readable, in form of a junit
        report, etc.)
        """

        failure = False
        timeout = False
        result = "SUCCESS"

        if self.scenario.test_criteria is None:
            print("Nothing to analyze, this scenario has no criteria")
            return True

        for criterion in self.scenario.get_criteria():
            if (not criterion.optional and
                    criterion.test_status != "SUCCESS" and
                    criterion.test_status != "ACCEPTABLE"):
                failure = True
                result = "FAILURE"
            elif criterion.test_status == "ACCEPTABLE":
                result = "ACCEPTABLE"

        if self.scenario.timeout_node.timeout and not failure:
            timeout = True
            result = "TIMEOUT"

        output = ResultOutputProvider(self, result, stdout, filename, junit, json)
        output.write()

        return failure or timeout
