import sys
try:
    sys.path.append('../../dependencies/CARLA_0.9.11/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg')
except IndexError:
    pass
import time
print('hoola-10')
import carla
print('haha')
import numpy as np
import argparse
import logging
from numpy import random
from subprocess import Popen, PIPE, STDOUT
import pickle, pickle5
import pandas as pd
import math
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (1920-150,1080-150)
import pygame

def collision_handler(event):
    print(event)

def add_collision_sensor(vehicle, world):
    """attaches a collision sensor to the vehicle

    Args:
        vehicle (carla.actor): vehicle
        world (carla.World): carla world

    Returns:
        carla.Actor: collision sensor that is attached to the vehicle
    """
    collision_sensor_bp = world.get_blueprint_library().find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_sensor_bp, carla.Transform(), attach_to=vehicle)
    collision_sensor.listen(lambda event: collision_handler(event))
    return collision_sensor

def attach_spectator(spectator, vehicle):
    """attaches the camera to the vehicle

    Args:
        spectator (carla.Spectator): camera of the carla client
        vehicle (carla.actor): vehicle
    """
    spectator_transform = vehicle.get_transform()
    spectator_transform.location += carla.Location(x=0, y=0, z=5.0)
    spectator_transform.rotation = carla.Rotation(pitch=-30, yaw=spectator_transform.rotation.yaw,
                                                  roll=spectator_transform.rotation.roll)
    spectator_transform.location += carla.Location(x=0, y=0, z=50.0)
    spectator_transform.rotation = carla.Rotation(pitch=-90, yaw=0, roll=0)
    spectator.set_transform(spectator_transform)

def set_spectator(spectator, world, scenario):
    if scenario == 1:
        points = world.get_map().get_spawn_points()
        spectator_transform  = points[69]
        # spectator_transform.location += carla.Location(x=12, y=-16, z=15.0)
        # spectator_transform.rotation = carla.Rotation(pitch=-40, yaw=45, roll=0)
        spectator_transform.location += carla.Location(x=30, y=15, z=15.0)
        spectator_transform.rotation = carla.Rotation(pitch=-40, yaw=-135, roll=0)
        spectator.set_transform(spectator_transform)
    elif scenario == 4:
        spectator_transform_location = carla.Location(x=165, y=55, z=7)
        spectator_transform_rotation = carla.Rotation(pitch=-25, yaw=160, roll=0)
        spectator_transform = carla.Transform(spectator_transform_location, spectator_transform_rotation)
        spectator.set_transform(spectator_transform)

def get_intersect(a1, a2, b1, b2):
    """calculated the intersection point between two lines

    Args:
        a1 (np.array): location of the first point on the first line
        a2 (np.array): location of the second point on the first line
        b1 (np.array): location of the first point on the second line
        b2 (np.array): location of the second point on the second line

    Returns:
        np.array: location of the intersection point
    """
    s = np.vstack([a1,a2,b1,b2])
    h = np.hstack((s, np.ones((4, 1))))
    l1 = np.cross(h[0], h[1])
    l2 = np.cross(h[2], h[3])
    x, y, z = np.cross(l1, l2)
    if z == 0:
        return False, None
    return True, np.array([x/z, y/z])

def normalize_2d(vect):
    """return the normalized vector

    Args:
        vect (np.array): 2D vector

    Returns:
        np.array: normalized 2D vector
    """
    if np.linalg.norm(vect) != 0:
        return vect / np.linalg.norm(vect)
    else:
        return np.array([0, 0])

def get_dist_to_intersection(inter_point, l_ego, l_o, ego_vel, other_vel=None):
    """calculated the distance from the ego_vehicle to the intersection point and
       from the to other vehicle to the intersection point,
       only considers the forward direction of the vehicles

    Args:
        inter_point (np.array): location of the intersection point
        l_ego (np.array): location of the ego_vehicle
        l_o (np.array): location of the other vehicle
        ego_vel (np.array): 2D vector of the velocity of the ego vehicle
        other_vel (np.array, optional): 2D vector of the velocity of the other vehicle
                                        (only used for vehicles not walkers). Defaults to None.

    Returns:
        [type]: [description]
    """
    v1 = normalize_2d(inter_point - l_ego)
    v2 = normalize_2d(inter_point - l_o)
    if ego_vel[0] > 0.005 and np.sign(v1[0]) != np.sign(ego_vel[0]):
        return MAX_DIST, MAX_DIST
    elif ego_vel[1] > 0.005 and np.sign(v1[1]) != np.sign(ego_vel[1]):
        return MAX_DIST, MAX_DIST
    if other_vel is not None and other_vel[0] > 0.005 and np.sign(v2[0]) != np.sign(other_vel[0]):
        return MAX_DIST, MAX_DIST
    elif other_vel is not None and other_vel[1] > 0.005 and np.sign(v2[1]) != np.sign(other_vel[1]):
        return MAX_DIST, MAX_DIST
    return np.linalg.norm(inter_point - l_ego), np.linalg.norm(inter_point - l_o)

def get_dir_vector_of_actor(actor):
    """returns the 2D vector of the velocity of the actor, if the velocity is very small the orientation of the car is used to determine the direction

    Args:
        actor (carla.Actor): vehicle that the direction should be calculated of

    Returns:
        np.array: 2D vector of the direction
    """
    vel = actor.get_velocity()
    vel = np.array([vel.x, vel.y])
    if np.linalg.norm(vel) > 0.05:
        return normalize_2d(vel)
    else:
        r_actor = actor.get_transform().rotation
        theta = np.radians(r_actor.yaw)
        rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        return np.dot(rot, np.array([1,0]))

def calc_observations_vehicles(ego_vehicle, actors):
    """calculates the distances of the vehicle and the walker by intersecting the
       velocity vectors of the vehicles

    Args:
        ego_vehicle (carla.Actor): car that should be shielded
        actors (array): list of carla.Actors of the other vehicles

    Returns:
        float, array, array: distance of the ego_vehicle, distances of the other vehicles (float),
                             velocities of the other vehicles (float)
    """
    t_ego = ego_vehicle.get_transform()
    l_ego = t_ego.location
    l_ego = np.array([l_ego.x, l_ego.y])
    ego_dir = get_dir_vector_of_actor(ego_vehicle)

    dists_ego = []
    dists_other = []
    other_vels = []
    for v in actors:
            t_o = v.get_transform()
            l_o = t_o.location
            l_o = np.array([l_o.x, l_o.y])
            other_vel = v.get_velocity()
            other_vels.append(np.linalg.norm(np.array([other_vel.x, other_vel.y])))
            other_dir = get_dir_vector_of_actor(v)
            inter, inter_point = get_intersect(l_ego, l_ego + ego_dir, l_o, l_o + other_dir)
            if inter:
                dist_ego, dist_other = get_dist_to_intersection(inter_point, l_ego, l_o, ego_dir, other_dir)
                dists_ego.append(dist_ego)
                dists_other.append(dist_other)
            else:
                dists_ego.append(MAX_DIST)
                dists_other.append(MAX_DIST)
    return dists_ego, dists_other, other_vels

def calc_observations_walkers(ego_vehicle, actors):
    """calculates the distances of the vehicle and the walkers by projecting the vector
       from the vehicle to the walker onto the velocity vector of the vehicle

    Args:
        ego_vehicle (carla.Actor): car that should be shielded
        actors (array): list of carla.Actors of the walkers

    Returns:
        float, array, array: distance of the ego_vehicle, distances of the walkers (float),
                             velocities of the walkers (float)
    """
    t_ego = ego_vehicle.get_transform()
    l_ego = t_ego.location
    l_ego = np.array([l_ego.x, l_ego.y])
    ego_dir = get_dir_vector_of_actor(ego_vehicle)

    dists_ego = []
    dists_other = []
    other_vels = []
    for v in actors:
            t_o = v.get_transform()
            l_o = t_o.location
            l_o = np.array([l_o.x, l_o.y])
            other_vel = v.get_velocity()
            other_vels.append(np.linalg.norm(np.array([other_vel.x, other_vel.y])))
            other_vel = normalize_2d(np.array([other_vel.x, other_vel.y]))
            e_o_vect = l_o - l_ego
            o_proj_vect = ego_dir * np.dot(e_o_vect, ego_dir)
            inter_point = l_ego + o_proj_vect
            dist_ego, dist_other = get_dist_to_intersection(inter_point, l_ego, l_o, ego_dir)
            dists_ego.append(dist_ego)
            dists_other.append(dist_other)

    return dists_ego, dists_other, other_vels

def is_obs_outside_of_shield(new_obs, delay, pedestrian_shield):
    if not pedestrian_shield:
        return False
    violate_spec = False
    
    for i in range(new_obs["other_amount"]):
        ego_dist = int(math.floor(new_obs["ego_dist"][i]))
        ego_vel = 0
        if new_obs['ego_vel'] > 0.05:
            ego_vel = int(math.ceil(new_obs["ego_vel"]))
        other_dist = int(math.floor(new_obs["other_dist"][i]))
        SAFETY_VEL = 2
        SAFETY_DIST = 3
        violate_spec = (ego_vel > SAFETY_VEL) and (ego_dist <= SAFETY_DIST) and (ego_dist >= other_dist)
        if violate_spec:
            return True
    return violate_spec


def calc_observations(obs, delay, ego_vehicle, world, vehicles_list, walkers_list, pedestrian_shield = False):
    """calculates the observations for the vehicles and walkers in the carla world
       calls the add_delay function in order to save the previous observations

    Args:
        obs (array): array of dicts with all the relevant informations to determine the shield state
        delay (int): amount of delay of the shield
        ego_vehicle (carla.Actor): the car that should be shielded
        world (carla.world): carla world
        vehicles_list (array): list of ids of all the vehicles
        walkers_list (array): list of ids of all the walkers

    Returns:
        array: array of dicts with all the relevant informations to determine the shield state
    """
    actors_vehicle = []
    if len(vehicles_list) > 1:
        actors_vehicle.extend(world.get_actors(vehicles_list[1:])
                        if len(vehicles_list) > 2 else [world.get_actor(vehicles_list[1])])
        dists_ego, dists_other, other_vels = calc_observations_vehicles(ego_vehicle, actors_vehicle)

    actors_walker = []
    if walkers_list:
        actors_walker.extend(world.get_actors([w["id"] for w in walkers_list])
                        if len(walkers_list) > 1 else [world.get_actor(walkers_list[0]["id"])])
        dists_ego, dists_other, other_vels = calc_observations_walkers(ego_vehicle, actors_walker)

    control = ego_vehicle.get_control()
    ego_vel = ego_vehicle.get_velocity()
    ego_vel = np.linalg.norm(np.array([ego_vel.x, ego_vel.y]))

    new_obs =  {"ego_vel": ego_vel,
                "ego_dist": np.minimum(dists_ego, MAX_DIST),
                "other_dist": np.minimum(dists_other, MAX_DIST),
                "other_vel": other_vels,
                "other_amount": len(dists_other),
                "ego_throttle": control.throttle,
                "ego_brake": control.brake}
    if is_obs_outside_of_shield(new_obs, delay, pedestrian_shield):
        print("Help!")
    return add_delay(obs, delay+2, new_obs)

def print_observations(obs):
    for o_i, o in enumerate(obs):
        print(f"obs {o_i-(len(obs)-1)}: ", end="")
        for n, i in o.items():
            if n == "ego_dist" or n == "other_dist" or n == "other_vel":
                print(f"{n} : ", end="")
                for j in i:
                    print(f"{j:.2f} ", end="")
                print("| ", end="")
            else:
                print(f"{n} : {i} | ", end="")
        print()

def add_delay(obs, delay, new_obs):
    """saves the previous observations according to the delay amount

    Args:
        obs (array): array of dicts with all the relevant informations to determine the shield state
        delay (int): amount of delay of the shield
        new_obs (dict): observation of the current time step

    Returns:
        array: array of dict with the new_obs added
    """
    if len(obs) < delay:
        obs.append(new_obs.copy())
        return obs
    else:
        obs = obs[1:]
        obs.append(new_obs.copy())
        return obs

def set_new_obs(obs, control, delay, shield_takeover):
    """sets the observations if the shield took over in order to save the correct actions

    Args:
        obs (array): array of dicts with all the relevant informations to determine the shield state
        control (carla.Control): control of the vehicle
        delay (int): amount of delay of the shield
        shield_takeover (bool): if the shield took over control in this time step

    Returns:
        array: array of dicts with the corrected observations
    """
    obs[delay+1]["ego_throttle"] = control.throttle
    obs[delay+1]["ego_brake"] = control.brake
    obs[delay+1]["shield_takeover"] = shield_takeover
    return obs

def get_action_from_obs(obs):
    """returns the action for the shield from an observations

    Args:
        obs (dict): array of dicts with all the relevant informations to determine the shield state

    Returns:
        string: action according to the throttle and brake value of the control of the car
    """
    t = obs["ego_throttle"]
    b = obs["ego_brake"]
    if t > 0.2:
        return "A"
    if b > 0.2:
        return "D"
    else:
        return "S"

def round_to_even(f):
    return int(round(f / 2.) * 2)

def round_up_to_even(f):
    return int(math.ceil(f / 2.) * 2)

def round_down_to_even(f):
    return int(math.floor(f / 2.) * 2)

def translate_obs_to_shield(obs, delay, pedestrian_shield = False):
    """translates the current observation to the shield observation
       crossing shield:
            <ego_dist>_<ego_velocity>_<other_dist>_<other_velocity>_<delay_actions>
       pedestrian shield:
            <ego_dist>_<ego_velocity>_<other_dist>_<delay_actions>
       actions = [D, S, A]

    Args:
        obs (array): array of dicts with all the relevant informations to determine the shield state
        delay (int): amount of delay of the shield
        pedestrian_shield (bool, optional): If the shield is a pedestrian shield. Defaults to False.

    Returns:
        [type]: [description]
    """
    if len(obs) != delay + 2:
            return None
    shield_obs = []
    for i in range(obs[1]["other_amount"]):
        s = ""
        if pedestrian_shield:
            s += str(int(math.floor(obs[1]["ego_dist"][i])))+"_"
        else:
            s += str(round_down_to_even(obs[1]["ego_dist"][i]))+"_"

        if obs[1]["ego_vel"] <= 0.05:
            s += "0_"
        else:
            s += str(int(math.ceil(obs[1]["ego_vel"])))+"_"

        if pedestrian_shield:
            s += str(int(math.floor(obs[1]["other_dist"][i])))
        else:
            s += str(round_down_to_even(obs[1]["other_dist"][i]))+"_"

        if not pedestrian_shield:
            if obs[1]["other_vel"][i] <= 0.05:
                s += "0"
            else:
                s += str(int(math.ceil(obs[1]["other_vel"][i])))

        if int(math.ceil(obs[1]["ego_vel"])) <= 8:
            if obs[0]["ego_throttle"] > 0.2:
                s += '_y'
            else:
                s += '_n'

        for i in range(delay):
            s += "_" + get_action_from_obs(obs[i+1])
        shield_obs.append(s)

    return shield_obs

def shield_car(shield, ego_vehicle, shield_obs, tm_port):
    """checks if the observation is in the shield and allowed,
       if not the shield takes over the control of the car

    Args:
        shield (dict): dict of the shield states and the allowed actions
        ego_vehicle (carla.Actor): car that is being shielded
        shield_obs (string): state of the observations
        tm_port (int): traffic manager port

    Returns:
        shield_takeover, control: returns if the shield had to take action and the control action for the car
    """
    shield_takeover = np.array([False for _ in range(len(shield_obs))])
    control = ego_vehicle.get_control()
    all_actions = np.empty((len(shield_obs), 3), dtype=bool)
    for i, s in enumerate(shield_obs):
        if s not in shield:
            print(f"observation {s} not in shield")
            # assert False, f"observation {s} not in shield"
            actions= np.array([True, True, True])
            all_actions[i] = actions
            shield_takeover[i] = True
        else:
            actions = shield[s]
            all_actions[i] = actions
            a = get_action_from_obs({"ego_throttle": control.throttle, "ego_brake": control.brake})
            if a == "D" and not actions[0]:
                shield_takeover[i] = True
            if a == "S" and not actions[1]:
                shield_takeover[i] = True
            if a == "A" and not actions[2]:
                shield_takeover[i] = True
    all_actions = all_actions.T

    if shield_takeover.any():
        print(f"*********************\n!! shield active !!\n******************")
        set_autopilot(ego_vehicle, False, tm_port)
        if np.all(all_actions[2]):
            control.brake = 0.0
            control.throttle = 0.4
        elif np.all(all_actions[1]):
            control.brake = 0.0
            control.throttle = 0.0
        elif np.all(all_actions[0]):
            control.brake = 0.4
            control.throttle = 0.0
        else:
            a = np.argmin(np.count_nonzero(all_actions))
            if a == 2:
                control.brake = 0.0
                control.throttle = 0.4
            elif a == 1:
                control.brake = 0.0
                control.throttle = 0.0
            elif a == 0:
                control.brake = 0.4
                control.throttle = 0.0

        ego_vehicle.apply_control(control)
    else:
        set_autopilot(ego_vehicle, True, tm_port)

    return shield_takeover, control

# def minishield(ego_vehicle, obs, tm_port):
#     shield_takeover = False
#     for d, a in zip(obs["dists"], obs["angles"]):
#         if d < 10.0 and a >= -45 and a <= +45:
#             control = ego_vehicle.get_control()
#             shield_takeover = True
#             if control.throttle > 0.0 or control.brake < 1.0:
#                 set_autopilot(ego_vehicle, False, tm_port)
#                 control.throttle = 0.0
#                 control.brake = 1.0
#                 control.hand_brake = True
#                 ego_vehicle.apply_control(control)
#             #print(f"throttle: {control.throttle}, brake: {control.brake}")
#             print(f"!! shield active !!")
#     if not shield_takeover:
#         set_autopilot(ego_vehicle, True, tm_port)
#     return shield_takeover

def draw_shield(screen, shield_img, shield_img_off, on):
    """draws the shield in a pygame window

    Args:
        screen (pygame.screen): pygame screen
        shield_img (pygame.image): image of the shield (on)
        shield_img_off (pygame.image): image of the shield (off)
        on (bool): if the shield should be in the on state or not
    """
    screen.fill([255, 255, 255])
    if on:
        screen.blit(shield_img, (0,0))
    else:
        screen.blit(shield_img_off, (0,0))
    pygame.display.flip()

def spawn_points_scenario(scenario, world):
    """returns the spawn points for the vehicles for a given scenario

    Args:
        scenario (int): number of scenario
        world (carla.World): carla world

    Returns:
        array: list of spawn points carla.Transform
    """
    points = world.get_map().get_spawn_points()
    if scenario == 0:
        return points
    elif scenario == 1:
        # 37, 38, 68, 69, 204, 206
        ego_point = points[69]
        ego_point_l = ego_point.location
        ego_point_r = ego_point.rotation
        ego_point_l = carla.Location(ego_point_l.x-45, ego_point_l.y, ego_point_l.z)
        ego_point = carla.Transform(ego_point_l, ego_point_r)
        other_point = points[38]
        other_point_l = other_point.location
        other_point_r = other_point.rotation
        other_point_l = carla.Location(other_point_l.x, other_point_l.y-(45), other_point_l.z)
        other_point = carla.Transform(other_point_l, other_point_r)
        return [ego_point, other_point]
    elif scenario == 2:
        ego_point = points[69]
        ego_point = points[69]
        ego_point_l = ego_point.location
        ego_point_r = ego_point.rotation
        ego_point_l = carla.Location(ego_point_l.x-10, ego_point_l.y, ego_point_l.z)
        ego_point = carla.Transform(ego_point_l, ego_point_r)
        other_point = points[38]
        other_point_l = other_point.location
        other_point_r = other_point.rotation
        other_point_l = carla.Location(other_point_l.x, other_point_l.y-25, other_point_l.z)
        other_point = carla.Transform(other_point_l, other_point_r)
        return [ego_point, other_point]
    elif scenario == 3:
        #38,
        ego_point = points[38]
        ego_point_l = ego_point.location
        ego_point_r = ego_point.rotation
        other_point_l = carla.Location(ego_point_l.x+5, ego_point_l.y-10, ego_point_l.z)
        other_point_r = carla.Rotation(ego_point_r.pitch, ego_point_r.yaw-15, ego_point_r.roll)
        other_point = carla.Transform(other_point_l, other_point_r)
        return [points[38], other_point]
    elif scenario == 4:
        # 87, 175, 201
        return [points[201]]
    else:
        return []

def spawn_vehicles(world, client, spawn_points, blueprints, traffic_manager, number_of_vehicles):
    """spawns the vehicles at the given locations

    Args:
        world (carla.World): carla world
        client (carla.Client): carla client
        spawn_points (array): list of spawn points carla.Transform
        blueprints (array): list of blueprints for the vehicles
        traffic_manager (carla.TrafficManager): traffic manager
        number_of_vehicles (int): number of vehicles that should be spawned

    Returns:
        [type]: [description]
    """
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    vehicles_list = []
    batch = []
    for n, transform in enumerate(spawn_points):
        if n >= number_of_vehicles:
            break
        blueprint = blueprints[n]
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')
        batch.append(SpawnActor(blueprint, transform)
                        .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)
    return vehicles_list

def walker_spawn_points_scenario(scenario, world):
    """sets the walker spawn points for scenario 4

    Args:
        scenario (int): number of scenario
        world (carla.World): carla world

    Returns:
        array: returns the list of spawn points carla.Transform if the scenario is 4,
               else None
    """
    if scenario == 4:
        spawn_points = []
        for _ in range(20):
            y_val = np.random.random() * 2 - 1 + 72
            x_val = np.random.random() * 7 - 3.5 + 150
            l = carla.Transform(carla.Location(x=x_val, y=y_val, z=0.5), carla.Rotation())
            spawn_points.append(l)
        return spawn_points
    else:
        return None

def spawn_walkers(world, client, walker_spawn_points):
    """spawns all the walkers at the given spawn points

    Args:
        world (carla.World): carla world
        client (carla.Client): carla client
        walker_spawn_points (array): array of carla.Transform, where the walkers should be spawned

    Returns:
        array: list of walker ids
    """
    SpawnActor = carla.command.SpawnActor
    blueprints_walkers = world.get_blueprint_library().filter('walker.pedestrian.*')
    blueprints_walkers = [w for w in blueprints_walkers if w.get_attribute('age').as_str() == "adult"]
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    batch = []
    walker_speed = []
    walkers_list = []
    for walker_spawn_point in walker_spawn_points:
        walker_bp = random.choice(blueprints_walkers)
        walker_bp.set_attribute('is_invincible', 'false')
        walker_speed.append(np.random.random())
        batch.append(SpawnActor(walker_bp, walker_spawn_point))
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            continue
        else:
            walkers_list.append({"id": response.actor_id})
    batch = []
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    for i, response in enumerate(client.apply_batch_sync(batch, True)):
        if response.error:
            logging.error(response.error)
        else:
            walkers_list[i]["ai"] = response.actor_id
    return walkers_list

def walker_ai(world, walkers_list):
    """sets the walker ai for all the walkers

    Args:
        world (carla.World): carla world
        walkers_list (array): list of all the walker ids
    """
    for walker in walkers_list:
        ai = world.get_actor(walker["ai"])
        ai.start()
        while True:
            y_val = np.random.random() * 2 - 1 + 50
            x_val = np.random.random() * 1 - 0.5 + 153
            l = carla.Location(x=x_val, y=y_val, z=0.5)
            waypoint = world.get_map().get_waypoint(l, project_to_road=False, lane_type=carla.LaneType.Sidewalk)
            if waypoint != None and len(waypoint.next(15)) != 0:
                break
        ai.go_to_location(waypoint.next(15)[0].transform.location)

def blueprint_scenario(scenario, world):
    """returns the blueprints depending on the current situation

    Args:
        scenario (int): number of the scenario
        world (carla.World): carla world

    Returns:
        array of blueprints: list of all the blueprints to use
    """
    if scenario == 0:
        blueprints = world.get_blueprint_library().filter("vehicle.*")
        blueprints = sorted(blueprints, key=lambda bp: bp.id)
        return blueprints.shuffle()
    elif scenario == 1 or scenario == 2:
        blueprint_library = world.get_blueprint_library()
        return [blueprint_library.find("vehicle.audi.etron"), blueprint_library.find("vehicle.toyota.prius")]
    elif scenario == 3:
        blueprint_library = world.get_blueprint_library()
        return [blueprint_library.find("vehicle.audi.etron"), blueprint_library.find("vehicle.toyota.prius")]
    elif scenario == 4:
        blueprint_library = world.get_blueprint_library()
        return [blueprint_library.find("vehicle.audi.etron")]

def set_traffic_manager(traffic_manager, vehicles_list, world, scenario):
    """sets the rules for the traffic manager depending on the current scenario

    Args:
        traffic_manager (carla.TrafficManager): traffic manager that controls the cars
        vehicles_list (array): list of the ids of all the vehicles
        world (carla.World): carla world
        scenario (int): number of the scenario
    """
    ego_vehicle = world.get_actor(vehicles_list[0])
    if scenario == 0:
        for v in world.get_actors(vehicles_list):
            traffic_manager.ignore_vehicles_percentage(v, 100)
            traffic_manager.ignore_signs_percentage(v, 100)
            traffic_manager.ignore_lights_percentage(v, 100)
            traffic_manager.distance_to_leading_vehicle(v, 0)
            traffic_manager.vehicle_percentage_speed_difference(v, -20)

    elif scenario == 1:
        other_vehicle = world.get_actor(vehicles_list[1])
        traffic_manager.ignore_vehicles_percentage(other_vehicle, 100)
        #traffic_manager.ignore_signs_percentage(other_vehicle, 100)
        traffic_manager.ignore_lights_percentage(other_vehicle, 100)
        traffic_manager.distance_to_leading_vehicle(other_vehicle, 0)
        traffic_manager.vehicle_percentage_speed_difference(other_vehicle, -10)
        traffic_manager.ignore_vehicles_percentage(ego_vehicle, 100)
        traffic_manager.ignore_lights_percentage(ego_vehicle, 100)
        traffic_manager.vehicle_percentage_speed_difference(ego_vehicle, -10)

    elif scenario == 2:
        other_vehicle = world.get_actor(vehicles_list[1])
        traffic_manager.ignore_vehicles_percentage(other_vehicle, 100)
        traffic_manager.ignore_lights_percentage(other_vehicle, 100)
        traffic_manager.distance_to_leading_vehicle(other_vehicle, 0)
        traffic_manager.vehicle_percentage_speed_difference(other_vehicle, 0)
        traffic_manager.ignore_vehicles_percentage(ego_vehicle, 100)
        traffic_manager.ignore_lights_percentage(ego_vehicle, 100)

    elif scenario == 3:
        other_vehicle = world.get_actor(vehicles_list[1])
        traffic_manager.ignore_vehicles_percentage(other_vehicle, 100)
        traffic_manager.distance_to_leading_vehicle(other_vehicle, 0)
        traffic_manager.vehicle_percentage_speed_difference(other_vehicle, -10)
        traffic_manager.ignore_vehicles_percentage(ego_vehicle, 100)
        traffic_manager.distance_to_leading_vehicle(ego_vehicle, 0)
        traffic_manager.vehicle_percentage_speed_difference(ego_vehicle, -10)

    elif scenario == 4:
        traffic_manager.ignore_walkers_percentage(ego_vehicle, 100)
        traffic_manager.ignore_signs_percentage(ego_vehicle, 100)
        traffic_manager.vehicle_percentage_speed_difference(ego_vehicle, -50)

def set_autopilot(ego_vehicle, activated, tm_port):
    """sets the autopilot

    Args:
        ego_vehicle (carla.Actor): vehicle that is controlled by the traffic manager
        activated (bool): True - car gets controlled by the traffic manager
        tm_port (int): port of the traffic manager
    """
    global EGO_AUTOPILOT
    if EGO_AUTOPILOT == activated:
        return
    ego_vehicle.set_autopilot(activated, tm_port)
    EGO_AUTOPILOT = activated

def set_traffic_light(vehicle, state):
    """sets the traffic light that affects the given vehicle

    Args:
        vehicle (carla.Actor): vehicle affected by a traffic light
        state (carla.TrafficLightState): state of the traffic light
    """
    traffic_light = vehicle.get_traffic_light()
    if traffic_light != None:
        traffic_light.set_state(state)

def start_recording():
    file_name = "output.mp4"
    try:
        os.remove(file_name)
    except OSError:
        pass
    cmd = "ffmpeg -video_size 1920x1080 -framerate 60 -f x11grab -i :0.0 " + file_name
    p = Popen("exec " + cmd, stdout=PIPE, stdin=PIPE, stderr=STDOUT, shell=True)
    logging.info("started recording")
    return p

def stop_recording(p):
    p.communicate(input=b'q')
    p.kill()
    logging.info("stopped recording")

def save_observations(filename, save_obs):
    """saved all the recorded observations to a file

    Args:
        filename (string): path to the file
        save_obs (array): array of all the observations
    """
    df = pd.DataFrame()
    for o in save_obs:
        df = df.append(o, ignore_index=True)
    df.to_csv(filename)
    print(f"observations saved to {filename}")

def load_shield(shield_name):
    """load the specified shield file

    Args:
        shield_name (string): path to the shield file

    Returns:
        dict: shield states and their allowed actions
    """
    with open(shield_name, "rb") as f:
        try:
            shield = pickle.load(f)
        except:
            shield = pickle5.load(f)
        # shield = pickle.load(f)
        print("shield loaded")
        return shield


def load_world(client, scenario):
    """load the specified world in the carla client

    Args:
        client: carla client
        scenario (int): number of scenario

    Returns:
        carla.World: world of the carla client
    """
    if scenario == 1 or scenario == 2 or scenario == 4:
        Town = "Town03"
    elif scenario == 3:
        Town = "Town07"

    current_world = client.get_world().get_map().name
    if (current_world != Town):
        return client.load_world('/Game/Carla/Maps/'+Town)
    else:
        return client.get_world()

"""number of timesteps the observations get calculated and the shield gets checked"""
EVERY_T_STEPS = 10
"""maximum distance the shield considers"""
MAX_DIST = 100
"""flag if the autopilot is currently active or not"""
EGO_AUTOPILOT = True

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int, help='TCP port to listen to (default: 2000)')
    argparser.add_argument('-n', '--number-of-vehicles', metavar='N', default=10, type=int, help='number of vehicles (default: 10)')
    argparser.add_argument('--tm-port', metavar='P', default=8000, type=int, help='port to communicate with TM (default: 8000)')
    argparser.add_argument('-s', '--seed', metavar='S', type=int, help='Random device seed')
    argparser.add_argument('--scenario', type=int, default=1, metavar = 'S', help='Which scenario to load')
    argparser.add_argument('--shield', default="", metavar='S', help='Which shield to load, if "" is given no shield will be used')
    argparser.add_argument('--save-obs', default="", metavar='S', help='Saving the observations (csv file); "" for no save file')
    argparser.add_argument('--delay', type=int, default=0, metavar='D', help='How much delay is used for the shield')
    argparser.add_argument('--record-screen', action='store_true', default=False, help='Record the screen (Linux)')
    argparser.add_argument('--display-shield', action='store_true', default=True, help='Display the shield sprite in an own window')
    argparser.add_argument('--compute-shield', action='store_true', default=False, help='Compute shield before starting. Not yet supported ')
    args = argparser.parse_args()

    if args.compute_shield:
        pass


    """pygame status window of the shield (on/off)"""
    if args.display_shield:
        pygame.init()
        pygame.display.set_caption('Shield')
        shield_size = 300
        screen = pygame.display.set_mode((shield_size, shield_size))
        print(f"./images/shield_{shield_size}x{shield_size}.png")
        shield_img = pygame.image.load(f"./images/shield_{shield_size}x{shield_size}.png")
        shield_img.convert()
        shield_img_off = pygame.image.load(f"./images/shield_{shield_size}x{shield_size}_off.png")
        shield_img_off.convert()
        pygame.display.set_icon(shield_img)
        draw_shield(screen, shield_img, shield_img_off, False)

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    if args.scenario > 4 or args.scenario < 0:
        logging.error("scenario not support choose a value between 1 to 4")
        exit(-1)

    pedestrian_shield = False
    vehicles_list = []
    walkers_list = []
    save_obs = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    """seed for the different scenarios"""
    if args.scenario == 1:
        args.seed = 4
    if args.scenario == 2:
        args.seed = 1
    if args.scenario == 3:
        args.seed = 7
    if args.scenario == 4:
        args.seed = 14
        pedestrian_shield = True
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        if args.scenario != 0:
            world = load_world(client, args.scenario)
        else:
            world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        """set number of vehicles for the different scenarios"""
        if args.scenario != 0:
            if args.scenario == 1 or args.scenario == 2 or args.scenario == 3:
                args.number_of_vehicles = 2
            if args.scenario == 4:
                args.number_of_vehicles = 1

        """set world setting (synchronoud, 1/20 time delta)"""
        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1/20
        world.apply_settings(settings)

        """spawn vehicles"""
        blueprints = blueprint_scenario(args.scenario, world)
        spawn_points = spawn_points_scenario(args.scenario, world)
        number_of_spawn_points = len(spawn_points)
        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points
        vehicles_list = spawn_vehicles(world, client, spawn_points, blueprints, traffic_manager, args.number_of_vehicles)
        ego_vehicle = world.get_actor(vehicles_list[0])

        """spawn walkers"""
        walker_spawn_points = walker_spawn_points_scenario(args.scenario, world)
        walkers_list = []
        if walker_spawn_points != None:
            walkers_list = spawn_walkers(world, client, walker_spawn_points)
            walker_ai(world, walkers_list)

        set_traffic_manager(traffic_manager, vehicles_list, world, args.scenario)

        collision_sensor_e = add_collision_sensor(ego_vehicle, world)
        if args.scenario != 0 and args.scenario != 4:
            collision_sensor_o = add_collision_sensor(world.get_actor(vehicles_list[1]), world)

        print(f'spawned {len(vehicles_list)} vehicles and {len(walkers_list)} walkers , press Ctrl+C to exit.')

        if args.record_screen:
            recorder = start_recording()

        """load the shield"""
        shield = None
        if args.shield != "":
            shield = load_shield(args.shield)

        spectator = world.get_spectator()
        tick_count = 0
        obs = []
        shield_takeover = False
        shield_control = None
        set_spectator(spectator, world, args.scenario)
        framecounter = 0
        while True:
            framecounter += 1
            if framecounter > 600:
                assert False
            shield_takeover_new = False
            if tick_count % EVERY_T_STEPS == 0:
                """tick the world two times with enabled autopilot in order to
                   get the action of the traffic manager"""
                set_autopilot(ego_vehicle, True, args.tm_port)
                world.tick()
                # attach_spectator(spectator, ego_vehicle)
                # set_spectator(spectator, world, args.scenario)
                world.tick()
                # attach_spectator(spectator, ego_vehicle)
                # set_spectator(spectator, world, args.scenario)
                """set traffic lights"""
                if args.scenario == 1 or args.scenario == 2:
                    other_vehicle = world.get_actor(vehicles_list[1])
                    set_traffic_light(ego_vehicle, carla.TrafficLightState.Green)
                    set_traffic_light(other_vehicle, carla.TrafficLightState.Red)
                """scenario 3, let the front car brake after 400 time steps"""
                if args.scenario == 3:
                    if tick_count == 400:
                        other_vehicle = world.get_actor(vehicles_list[1])
                        control = other_vehicle.get_control()
                        control.throttle = 0.0
                        control.brake = 1.0
                        other_vehicle.apply_control(control)
                        set_autopilot(other_vehicle, False, args.tm_port)
                    if tick_count == 450:
                        set_autopilot(other_vehicle, True, args.tm_port)

                """calculate the observations and shield the car if a shield is active"""
                obs = calc_observations(obs, args.delay, ego_vehicle, world, vehicles_list, walkers_list, pedestrian_shield=pedestrian_shield)
                shield_obs = translate_obs_to_shield(obs, args.delay, pedestrian_shield)
                print(shield_obs)
                time.sleep(0.1)
                if shield != None and shield_obs != None and tick_count >= 30:
                    shield_takeover_new, shield_control = shield_car(shield, ego_vehicle, shield_obs, args.tm_port)
                    obs = set_new_obs(obs, shield_control, args.delay, shield_takeover_new)
                    shield_takeover_new = shield_takeover_new.any()
                print_observations(obs)

                """draw the shield status"""
                if args.display_shield and shield_takeover_new != shield_takeover:
                    draw_shield(screen, shield_img, shield_img_off, shield_takeover_new)
                if shield_takeover_new  != shield_takeover:
                    shield_takeover = shield_takeover_new

                """append the observations to the observation list if we want to save them"""
                if args.save_obs != "":
                    save_obs.append(obs[-1])
                print()

            world.tick()
            """check if the action of the shield is applied to the control of the ego_vehicle"""
            if (shield_takeover and (ego_vehicle.get_control().throttle != shield_control.throttle or
                                     ego_vehicle.get_control().brake != shield_control.brake)):
                input("Action of the car does not match shield action; continue?")
            # attach_spectator(spectator, ego_vehicle)
            # set_spectator(spectator, world, args.scenario)
            tick_count += 1

    finally:
        """clean up"""

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        print('destroying %d pedestrians' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x["id"]) for x in walkers_list])

        if args.record_screen:
            stop_recording(recorder)

        if args.display_shield:
            pygame.quit()

        if args.save_obs != "":
            save_observations(args.save_obs, save_obs)

        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        pass
