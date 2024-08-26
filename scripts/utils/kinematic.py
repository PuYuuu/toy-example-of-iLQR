import numpy as np

def kinematic_propagate(cur_x, cur_u, dt, wheelbase):
    # see: https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
    # implementation of <2.3. If the desired point is at the center of gravity or cg.>
    beta = np.arctan(np.tan(cur_u[1]) / 2)
    next_x = np.array([
        cur_x[0] + cur_x[2] * np.cos(beta + cur_x[3]) * dt,
        cur_x[1] + cur_x[2] * np.sin(beta + cur_x[3]) * dt,
        cur_x[2] + cur_u[0] * dt,
        cur_x[3] + 2 * cur_x[2] * np.sin(beta) * dt / wheelbase
    ])

    return next_x


def get_kinematic_model_derivatives(x, u, dt, wheelbase, N):
    N_velo = x[2, :-1]
    N_yaw = x[3, :-1]
    N_beta = np.arctan(np.tan(u[1] / 2))
    N_beta_over_stl = 0.5 * (1 + np.tan(u[1])**2) / (1 + 0.25 * np.tan(u[1])**2)

    # f(state, ctrl) over state of t_0 to t_N
    # df_dx.shape: (N, 4, 4)
    # For t_k, df_dx[k] is organized by:
    #   [[x_k+1     -> x_k, x_k+1     -> y_k, x_k+1     -> v_k, x_k+1     -> theta_k]
    #    [y_k+1     -> x_k, y_k+1     -> y_k, y_k+1     -> v_k, y_k+1     -> theta_k]
    #    [v_k+1     -> x_k, v_k+1     -> y_k, v_k+1     -> v_k, v_k+1     -> theta_k]
    #    [theta_k+1 -> x_k, theta_k+1 -> y_k, theta_k+1 -> v_k, theta_k+1 -> theta_k]]
    df_dx = np.tile(np.eye(4), (N, 1, 1))
    df_dx[:, 0, 2] = np.cos(N_beta + N_yaw) * dt
    df_dx[:, 0, 3] = N_velo * (-np.sin(N_beta + N_yaw)) * dt
    df_dx[:, 1, 2] = np.sin(N_beta + N_yaw) * dt
    df_dx[:, 1, 3] = N_velo * np.cos(N_beta + N_yaw) * dt
    df_dx[:, 3, 2] = 2 * np.sin(N_beta) * dt / wheelbase

    # f(state, ctrl) over ctrl of t_0 to t_N
    # df_du.shape: (N, 4, 2)
    # For t_k, df_du[k] is organized by:
    #   [[x_k+1     -> a_k, x_k+1     -> delta_k]
    #    [y_k+1     -> a_k, y_k+1     -> delta_k]
    #    [v_k+1     -> a_k, v_k+1     -> delta_k]
    #    [theta_k+1 -> a_k, theta_k+1 -> delta_k]]
    df_du = np.zeros((N, 4, 2))
    df_du[:, 2, 0] = np.ones(N) * dt
    df_du[:, 0, 1] = N_velo * (-np.sin(N_beta + N_yaw)) * dt * N_beta_over_stl
    df_du[:, 1, 1] = N_velo * np.cos(N_beta + N_yaw) * dt * N_beta_over_stl
    df_du[:, 3, 1] = (2 * N_velo * dt / wheelbase) * np.cos(N_beta) * N_beta_over_stl

    # Put time horizon at the innermost dimension to satisfy the output formats
    return df_dx.transpose(1, 2, 0), df_du.transpose(1, 2, 0)


def const_velo_prediction(x0, steps: int, dt: float, wheelbase: float) -> np.matrix:
    cur_u = np.zeros(2)

    predicted_states = [x0]
    cur_x = x0
    for i in range(steps):
        next_x = kinematic_propagate(cur_x, cur_u, dt, wheelbase)
        cur_x = next_x
        predicted_states.append(next_x)

    predicted_states = np.vstack(predicted_states).transpose()

    return predicted_states


def get_ref_exact_points(pos, ref_waypoints):
    ref_waypoints_reshaped = ref_waypoints.transpose()[:, :, np.newaxis]
    distances = np.sum((pos - ref_waypoints_reshaped) ** 2, axis = 1)
    arg_min_dist_indices = np.argmin(distances, axis = 0)
    ref_exact_points = ref_waypoints[:, arg_min_dist_indices]

    return ref_exact_points


def get_vehicle_front_and_rear_centers(pos, yaw, wheelbase):
    half_whba_vec = 0.5 * wheelbase * np.array([np.cos(yaw), np.sin(yaw)])
    front_pnt = pos + half_whba_vec
    rear_pnt = pos - half_whba_vec

    return front_pnt, rear_pnt


def get_vehicle_front_and_rear_center_derivatives(yaw, wheelbase):
    half_whba = 0.5 * wheelbase

    # front point over (center) state:
    #           [[x_fr -> x_c, x_fr -> y_c, x_fr -> v, x_fr -> yaw]
    #            [y_fr -> x_c, y_fr -> y_c, y_fr -> v, y_fr -> yaw]]
    front_pnt_over_state = np.array([
        [1., 0., 0., half_whba * (-np.sin(yaw))],
        [0., 1., 0., half_whba *   np.cos(yaw) ]
    ])

    # rear point over (center) state:
    #            <similarly...>
    rear_point_over_state = np.array([
        [1., 0., 0., -half_whba * (-np.sin(yaw))],
        [0., 1., 0., -half_whba *   np.cos(yaw) ]
    ])

    return front_pnt_over_state, rear_point_over_state


def get_ellipsoid_obstacle_scales(ego_pnt_radius, obs_width, obs_length, d_safe):
    a = 0.5 * obs_length + d_safe + ego_pnt_radius
    b = 0.5 * obs_width + d_safe + ego_pnt_radius

    return a, b


def ellipsoid_safety_margin(pnt, elp_center, theta, a, b):
    diff = pnt - elp_center
    rotation_matrx = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    pnt_std = diff @ rotation_matrx  # rotate by (-theta)

    result = 1 - ((pnt_std[0] ** 2) / (a ** 2) + (pnt_std[1] ** 2) / (b ** 2))

    return result


def ellipsoid_safety_margin_derivatives(pnt, elp_center, theta, a, b):
    diff = pnt - elp_center
    rotation_matrx = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    pnt_std = diff @ rotation_matrx  # rotate by (-theta)

    # (1) constraint over standard point vec.:
    #       [c -> x_std, c -> y_std]
    res_over_pnt_std = np.array([-2 * pnt_std[0] / (a ** 2), -2 * pnt_std[1] / (b ** 2)])

    # (2) standard point vec. over difference vec.:
    #       [[x_std -> x_diff, x_std -> y_diff]
    #        [y_std -> x_diff, y_std -> y_diff]]
    pnt_std_over_diff = rotation_matrx.transpose()

    # (3) difference vec. over original point vec.:
    #       [[x_diff -> x, x_diff -> y]
    #        [y_diff -> x, y_diff -> y]]
    diff_over_pnt = np.eye(2)

    # chain (1)(2)(3) together:
    #       [c -> x, c -> y]
    res_over_pnt = res_over_pnt_std @ pnt_std_over_diff @ diff_over_pnt

    return res_over_pnt
