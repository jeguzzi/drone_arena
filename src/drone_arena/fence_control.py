import math

import numpy as np


def inside_fence_margin(position, fence, margin):
    return all([x > x_min - margin and x < x_max + margin
                for x, (x_min, x_max)
                in zip(position[:2], fence[:2])])


def desired_angular_speed(rotation_tau, yaw, target_yaw):
    d_yaw = target_yaw - yaw
    if d_yaw > math.pi:
        d_yaw = d_yaw - 2 * math.pi
    if d_yaw < -math.pi:
        d_yaw = d_yaw + 2 * math.pi
    return d_yaw / rotation_tau


def desired_velocity(eta, position, velocity, target_position, target_velocity=None, delay=0):
    if target_velocity is None:
        target_velocity = [0, 0, 0]
    return ((np.array(target_position) - np.array(position) - np.array(velocity) * delay) / eta +
            np.array(target_velocity))


def desired_acceleration(tau, velocity, target_velocity):
    v_t = np.array(target_velocity[:2])
    return (v_t - np.array(velocity[:2])) / tau


def angular_control(yaw, target_yaw=0, target_angular_speed=None, max_angular_speed=1,
                    rotation_tau=0.5):
    if target_angular_speed is not None:
        des_angular_speed = target_angular_speed
    else:
        des_angular_speed = desired_angular_speed(rotation_tau, yaw, target_yaw)
    des_angular_speed = clamp(target_angular_speed, -max_angular_speed, max_angular_speed)
    return (target_yaw, des_angular_speed)


def clamp(x, a, b):
    return max(min(x, b), a)


def clamp_v(xs, bss):
    return [clamp(x, *bs) for x, bs in zip(xs, bss)]


def clamp_h_norm(vector, value):
    copy = np.array(vector)
    h_vector = vector[:2]
    norm = np.linalg.norm(h_vector)
    if norm > value:
        copy[:2] = h_vector / norm * value
    return copy


def _v(x_t, x, v, eta, delay):
    return (x_t - x - v * delay) / eta


def velocity_bounds(eta, position, velocity, fence, delay=0):
    return [[_v(b, p, v, eta, delay) for b in bs] for bs, p, v in zip(fence, position, velocity)]


def _a(x_t, x, v, tau, eta, delay):
    return ((x_t - x - v * delay) / eta - v) / tau


def acceleration_bounds(eta, tau, position_bounds, position, velocity, delay=0,
                        maximal_acceleration=1):
    return [[_a(b, p, v, tau, eta, delay) for b in bs]
            for bs, p, v in zip(position_bounds, position, velocity)]


def fence_control(position, velocity,
                  target_position=None, target_velocity=None, target_acceleration=None,
                  delay=0, fence=None, maximal_acceleration=1, maximal_speed=1,
                  maximal_vertical_speed=1, eta=1, tau=0.5):
    des_position = des_velocity = des_acceleration = None
    if target_position is not None:
        if fence is not None:
            target_position = clamp_v(target_position, fence)
        else:
            des_position = target_position
    if des_position is None:
        des_velocity = target_velocity
    else:
        des_velocity = desired_velocity(eta, position, velocity, des_position, target_velocity,
                                        delay=delay)
    if des_velocity is not None:
        des_velocity = clamp_h_norm(des_velocity, maximal_speed)
        des_velocity[2] = clamp(des_velocity[2], -maximal_vertical_speed, maximal_vertical_speed)

    if target_velocity is not None and fence is not None:
        des_velocity = clamp_v(des_velocity, velocity_bounds(eta, position, velocity, fence,
                               delay=delay))
    if target_acceleration is None:
        des_acceleration = desired_acceleration(tau, velocity, des_velocity)
    else:
        des_acceleration = target_acceleration
        if fence is not None:
            des_acceleration = clamp_v(target_acceleration,
                                       acceleration_bounds(eta, tau, position, velocity, fence,
                                                           delay=delay))
    des_acceleration = clamp_h_norm(des_acceleration, maximal_acceleration)
    return (des_position, des_velocity, des_acceleration)
