import sys
import pathlib
import numpy as np

sys.path.append(str(pathlib.Path(__file__).parent))
import kinematic

def exp_barrier(c, q1, q2):
    b = q1 * np.exp(q2 * c)

    return b


def exp_barrier_derivative_and_Hessian(c, c_dot, q1, q2):
    b = exp_barrier(c, q1, q2)
    b_dot = q2 * b * c_dot
    c_dot_reshaped = c_dot[:, np.newaxis]
    b_ddot = (q2 ** 2) * b * (c_dot_reshaped @ c_dot_reshaped.T)

    return b_dot, b_ddot


def get_bound_constr(var, bound, bound_type='upper'):
    assert bound_type == 'upper' or bound_type == 'lower'

    if bound_type == 'upper':
        return var - bound
    else:
        return bound - var


# def get_bound_constr_derivative(bound_type='upper'):
#     assert bound_type == 'upper' or bound_type == 'lower'
#
#     if bound_type == 'upper':
#         return 1
#     else:
#         return -1


def get_obstacle_avoidance_constr(ego_state, obs_state, ego_wheelbase, ego_width, obs_attr):
    ego_front, ego_rear = kinematic.get_vehicle_front_and_rear_centers(ego_state[:2], ego_state[3], ego_wheelbase)
    obs_a, obs_b = kinematic.get_ellipsoid_obstacle_scales(0.5 * ego_width, obs_attr[0], obs_attr[1], obs_attr[2])
    front_safety_margin = kinematic.ellipsoid_safety_margin(ego_front, obs_state[:2], obs_state[3], obs_a, obs_b)
    rear_safety_margin = kinematic.ellipsoid_safety_margin(ego_rear, obs_state[:2], obs_state[3], obs_a, obs_b)

    return front_safety_margin, rear_safety_margin


def get_obstacle_avoidance_constr_derivatives(ego_state, obs_state, ego_wheelbase, ego_width, obs_attr):
    ego_front, ego_rear = kinematic.get_vehicle_front_and_rear_centers(ego_state[:2], ego_state[3], ego_wheelbase)
    obs_a, obs_b = kinematic.get_ellipsoid_obstacle_scales(0.5 * ego_width, obs_attr[0], obs_attr[1], obs_attr[2])

    # safety margin over ego front and rear points
    front_safety_margin_over_ego_front = kinematic.ellipsoid_safety_margin_derivatives(ego_front, obs_state[:2], obs_state[3], obs_a, obs_b)
    rear_safety_margin_over_ego_rear = kinematic.ellipsoid_safety_margin_derivatives(ego_rear, obs_state[:2], obs_state[3], obs_a, obs_b)

    # ego front and rear points over state
    ego_front_over_state, ego_rear_over_state = kinematic.get_vehicle_front_and_rear_center_derivatives(ego_state[3], ego_wheelbase)

    # chain together
    front_safety_margin_over_state = front_safety_margin_over_ego_front @ ego_front_over_state
    rear_safety_margin_over_state = rear_safety_margin_over_ego_rear @ ego_rear_over_state

    return front_safety_margin_over_state, rear_safety_margin_over_state


def get_bounded_ctrl(cur_u, cur_x, acc_max, acc_min, stl_lim, velo_min, velo_max):
    # todo: calculate the bounded ctrl outputs based on ctrl bounds and velocity bounds
    # Currently, let's try not to apply this operation but simply return the original value.

    return cur_u