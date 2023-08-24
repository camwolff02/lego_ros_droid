"""Differential Drive Wheeled Mobile Robot (WMR) Kinematics"""
import math


def rpm_to_rad(rpm: float) -> float:
    """convert rotations per minute [rpm] to radians per second [rad/s]"""
    return math.pi * rpm / 30


def rad_to_rpm(rad_per_sec: float) -> float:
    """convert radians per second [rad/s] to rotations per minute [rpm]"""
    return 30 * rad_per_sec / math.pi


def ang_to_lin(angular_val: float, radius: float) -> float:
    """convert angle, angular velocity, or angular acceleration to linear
    using neutonian mechanics"""
    return angular_val * radius


def lin_to_ang(linear_val: float, radius: float) -> float:
    """convert distance, velocity, or acceleration to angular
    using neutonian mechanics"""
    return linear_val / radius


def diff_drive_ik(linear: float, angular: float, radius: float, separation: float) -> tuple[float, float]:
    """Inverse kinematics to transform WMR linear and angular velocities
    into left and right wheel velocities.

    :param linear: linear velocity [m/s]
    :param angular: angular velocity [rad/s]
    :param radius: wheel radius [m]
    :param separation: distance between wheels [m]
    :returns: left wheel velocity [rad/s], right wheel velocity [rad/s]
    """
    return linear + (angular * separation) / 4, linear - (angular * separation) / 4


def diff_drive_fk(left_vel: float, right_vel: float, radius: float, separation: float) -> tuple[float, float]:
    """Forward kinematics to transform left and right wheel velocities
    into WMR linear and angular velocities.

    :param left_vel: left wheel velocity [rad/s]
    :param right_vel: right wheel velocity [rad/s]
    :param radius: wheel radius [m]
    :param separation: wheel separation [m]
    :returns: linear velocity [m/s], angular velocity [rad/s]
    """
    return (right_vel+left_vel) / 2, (right_vel-left_vel) / separation 
