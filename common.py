import math
import numpy as np


def pi_2_pi(theta):
    while theta > math.pi or theta < -math.pi:

        if theta > math.pi:
            theta -= 2 * math.pi

        if theta < -math.pi:
            theta += 2 * math.pi

    return theta


def cal_angle(vec_1, vec_2):
    norm_1 = np.hypot(vec_1[0], vec_1[1])
    norm_2 = np.hypot(vec_2[0], vec_2[1])
    if norm_1*norm_2 != 0:
        cos_theta = (vec_1[0]*vec_2[0] + vec_1[1]*vec_2[1])/(norm_1*norm_2)
    else:
        return None
    return math.acos(cos_theta)


def ref_waypoint(wp, max_dist = 30, dist_rate = 1.4):
    start_dist = 1
    wp_l = []
    while True:
        wp_l.append(wp.next(start_dist)[0])
        start_dist *= dist_rate
        if start_dist > max_dist:
            break
    return wp_l

