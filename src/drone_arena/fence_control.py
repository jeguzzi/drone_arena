import math

import numpy as np

from typing import List, Optional, Tuple  # noqa


def inside_fence_margin(position, fence, margin
                        ):  # type: (np.ndarray, List[Tuple[float, float]], float) -> bool
    return all([(x_min is None or x > x_min - margin) and (x_max is None or x < x_max + margin)
                for x, (x_min, x_max)
                in zip(position[:2], fence[:2])])


def desired_angular_speed(rotation_tau, yaw, target_yaw):  # type: (float, float, float) -> float
    d_yaw = target_yaw - yaw
    if d_yaw > math.pi:
        d_yaw = d_yaw - 2 * math.pi
    if d_yaw < -math.pi:
        d_yaw = d_yaw + 2 * math.pi
    # rospy.loginfo("d_yaw %s", d_yaw)
    return d_yaw / rotation_tau


def desired_velocity(eta, position, velocity, target_position, target_velocity=None, delay=0):
    # type: (float, np.ndarray, np.ndarray, np.ndarray, Optional[np.ndarray], float) -> np.ndarray
    if target_velocity is None:
        target_velocity = [0, 0, 0]
    return ((np.array(target_position) - np.array(position) - np.array(velocity) * delay) / eta +
            np.array(target_velocity))


def desired_acceleration(tau, velocity, target_velocity):
    # type: (float, np.ndarray, np.ndarray) -> np.ndarray
    v_t = np.array(target_velocity[:2])
    return (v_t - np.array(velocity[:2])) / tau


def angular_control(yaw, target_yaw=None, target_angular_speed=None, maximal_angular_speed=1,
                    rotation_tau=0.5):
    # type: (float, Optional[float], Optional[float], float, float) -> Tuple[float, float]
    if target_angular_speed is not None:
        des_angular_speed = target_angular_speed
    elif target_yaw is not None:
        des_angular_speed = desired_angular_speed(rotation_tau, yaw, target_yaw)
    else:
        des_angular_speed = 0
    des_angular_speed = clamp(des_angular_speed, -maximal_angular_speed, maximal_angular_speed)
    return (target_yaw or 0, des_angular_speed)


def clamp(x, a, b):
    # type: (float, float, float) -> float
    return max(min(x, b), a)


def clamp_v(xs, bss):
    # type: (List[float], List[Tuple[float, float]]) -> List[float]
    return [clamp(x, *bs) for x, bs in zip(xs, bss)]


def clamp_h_norm(vector, value):
    # type: (np.ndarray, float) -> np.ndarray
    copy = np.array(vector)
    h_vector = vector[:2]
    norm = np.linalg.norm(h_vector)
    if norm > value:
        copy[:2] = h_vector / norm * value
    return copy


def _v(x_t, x, v, eta, delay):
    # type: (np.ndarray, np.ndarray, np.ndarray, float, float) -> np.ndarray
    return (x_t - x - v * delay) / eta


def velocity_bounds(eta, position, velocity, fence, delay=0):
    # type: (float, np.ndarray,  np.ndarray, List[Tuple[float, float]], float) -> List
    return [[_v(b, p, v, eta, delay) for b in bs] for bs, p, v in zip(fence, position, velocity)]


def _a(x_t, x, v, tau, eta, delay):
    # type: (np.ndarray, np.ndarray, np.ndarray, float, float, float) -> np.ndarray
    return ((x_t - x - v * delay) / eta - v) / tau


def acceleration_bounds(eta, tau, position, velocity, position_bounds, delay=0,
                        maximal_acceleration=1):
    # type: (float, float, np.ndarray, np.ndarray, List[Tuple[float, float]], float, float)-> List
    return [[_a(b, p, v, tau, eta, delay) for b in bs]
            for bs, p, v in zip(position_bounds, position, velocity)]


def fence_control(position,  # type: np.ndarray
                  velocity,  # type: np.ndarray
                  target_position=None,  # type: Optional[np.ndarray]
                  target_velocity=None,  # type: Optional[np.ndarray]
                  target_acceleration=None,  # type: Optional[np.ndarray]
                  delay=0,  # type: float
                  fence=None,  # type: Optional[List]
                  maximal_acceleration=1,  # type: float
                  maximal_speed=1,  # type: float
                  maximal_vertical_speed=1,  # type: float
                  eta=1,  # type: float
                  tau=0.5,  # type: float
                  compute_velocity=True,  # type: bool
                  compute_acceleration=True  # type: bool
                  ):  # type: (...) -> Tuple[np.ndarray, np.ndarray, np.ndarray]
    des_position = des_velocity = des_acceleration = None
    if target_position is not None:
        if fence is not None:
            des_position = clamp_v(target_position, fence)
        else:
            des_position = target_position
    if not (compute_velocity or compute_acceleration):
        return (des_position, None, None)
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
    if not compute_acceleration:
        return (des_position, des_velocity, None)
    if target_acceleration is None:
        des_acceleration = desired_acceleration(tau, velocity, des_velocity)
    else:
        des_acceleration = target_acceleration
        if fence is not None:
            des_acceleration = clamp_v(target_acceleration,
                                       acceleration_bounds(eta, tau, position, velocity, fence,
                                                           delay=delay))

    # rospy.loginfo("target acceleration %s, des acceleration %s",
    #               target_acceleration, des_acceleration)
    des_acceleration = clamp_h_norm(des_acceleration, maximal_acceleration)
    return (des_position, des_velocity, des_acceleration)
