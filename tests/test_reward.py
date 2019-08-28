# -*- coding: utf-8 -*-

"""
This is the test code used to test implemented methods and function in the ../reward_function.py file. The set of
unit test is optional for you to use. You will not use it for purpose of training in AWS console.
"""

import math
import pytest

from parms.parms import get_copy_of_params as get_test_params
from reward_function import RewardEvaluator, MAX_STEERING_ANGLE, CENTERLINE_FOLLOW_RATIO_TRESHOLD


track_names = [None, "reinvent2018", "BOWTIE"]

@pytest.fixture()
def test_params():
    def _test_params(track_name):
        return get_test_params(track_name)
    return _test_params


@pytest.fixture()
def evaluator():
    def _re(track_name):
        return RewardEvaluator(get_test_params(track_name))
    return _re


@pytest.mark.parametrize("track_name", track_names)
def test_get_way_points_distance(evaluator, track_name):
    re = evaluator(track_name)
    assert re.get_way_points_distance((0, 0), (2, 0)) == 2
    assert re.get_way_points_distance((0, 0), (2, 2)) == math.sqrt(8)
    assert re.get_way_points_distance((-2, 4), (-4, 2)) == math.sqrt(8)
    assert re.get_way_points_distance((0, 0), (1, 0)) == 1


@pytest.mark.parametrize("track_name", track_names)
def test_get_heading_between_waypoints(evaluator, track_name):
    re = evaluator(track_name)
    assert re.get_heading_between_waypoints((0, 0), (2, 0)) == 0
    assert re.get_heading_between_waypoints((0, 0), (0, 2)) == 90
    assert re.get_heading_between_waypoints((0, 0), (0, -2)) == -90
    assert re.get_heading_between_waypoints((0, 0), (2, 2)) == 45
    assert re.get_heading_between_waypoints((0, 0), (-2, -2)) == -135


@pytest.mark.parametrize("track_name", track_names)
def test_get_car_heading_error(test_params, track_name):
    params_test = test_params(track_name)
    params_test['heading'] = 0
    params_test['waypoints'] = [(0, 0), (2, 0), (2, 2), (0, 2), (0, 0), (2, 2), (4, 0)]
    params_test['closest_waypoints'] = [0, 1]
    re = RewardEvaluator(params_test)
    assert re.get_car_heading_error() == 0
    params_test['closest_waypoints'] = [1, 2]
    re = RewardEvaluator(params_test)
    assert re.get_car_heading_error() == 90
    params_test['closest_waypoints'] = [2, 3]
    re = RewardEvaluator(params_test)
    assert re.get_car_heading_error() == 180
    params_test['closest_waypoints'] = [3, 4]
    re = RewardEvaluator(params_test)
    assert re.get_car_heading_error() == -90
    params_test['closest_waypoints'] = [4, 5]
    re = RewardEvaluator(params_test)
    assert re.get_car_heading_error() == 45
    params_test['closest_waypoints'] = [5, 6]
    re = RewardEvaluator(params_test)
    assert re.get_car_heading_error() == -45


@pytest.mark.parametrize("track_name,optimal_speed, speed_ratio", [
    (None, 1.0, 0.66),
    ("reinvent2018", 1.0, 0.66),
    ("BOWTIE", 0.34, 0.34)

])
def test_get_optimum_speed_ratio(test_params, track_name, optimal_speed, speed_ratio):
    params_test = test_params(track_name)
    params_test['heading'] = 0
    params_test['distance_from_center'] = 0
    params_test['steering_angle'] = 0
    params_test['closest_waypoints'] = (0, 1)
    params_test['x'] = params_test['waypoints'][params_test['closest_waypoints'][0]][0]
    params_test['y'] = params_test['waypoints'][params_test['closest_waypoints'][0]][1]
    re = RewardEvaluator(params_test)
    assert abs(re.get_optimum_speed_ratio()-optimal_speed) < 0.1
    params_test['closest_waypoints'] = (9, 10)
    params_test['x'] = params_test['waypoints'][params_test['closest_waypoints'][0]][0]
    params_test['y'] = params_test['waypoints'][params_test['closest_waypoints'][0]][1]
    re = RewardEvaluator(params_test)
    assert abs(re.get_optimum_speed_ratio()-speed_ratio) < 0.1
    params_test['closest_waypoints'] = (10, 11)
    params_test['x'] = params_test['waypoints'][params_test['closest_waypoints'][0]][0]
    params_test['y'] = params_test['waypoints'][params_test['closest_waypoints'][0]][1]
    re = RewardEvaluator(params_test)
    assert abs(re.get_optimum_speed_ratio()-0.33) < 0.1

    params_test = get_test_params()
    params_test['distance_from_center'] = 0
    params_test['steering_angle'] = 0
    params_test['closest_waypoints'] = (0, 1)
    params_test['x'] = params_test['waypoints'][params_test['closest_waypoints'][0]][0]
    params_test['y'] = params_test['waypoints'][params_test['closest_waypoints'][0]][1]
    params_test['heading'] = 1.1 * MAX_STEERING_ANGLE
    re = RewardEvaluator(params_test)
    assert re.get_optimum_speed_ratio() == 0.34
    params_test['heading'] = 1.1 * (MAX_STEERING_ANGLE * 0.75)
    re = RewardEvaluator(params_test)
    assert re.get_optimum_speed_ratio() == 0.67
    # print_get_optimum_speed_ratio()


@pytest.mark.parametrize("track_name", track_names)
def test_get_waypoint(evaluator, track_name):
    re = evaluator(track_name)
    re.waypoints = [(0, 0), (1, 0), (2, 0), (3, 3)]
    assert re.get_way_point(0) == (0, 0)
    assert re.get_way_point(1) == (1, 0)
    assert re.get_way_point(2) == (2, 0)
    assert re.get_way_point(3) == (3, 3)
    assert re.get_way_point(4) == (0, 0)
    assert re.get_way_point(5) == (1, 0)
    assert re.get_way_point(-1) == (3, 3)
    assert re.get_way_point(-2) == (2, 0)
    assert re.get_way_point(-3) == (1, 0)


@pytest.mark.parametrize("track_name", track_names)
def test_is_in_optimized_corridor(evaluator, track_name):
    re = evaluator(track_name)
    params_test = get_test_params()
    params_test['heading'] = 0
    params_test['track_width'] = 2
    params_test['distance_from_center'] = 0
    params_test['is_left_of_center'] = True
    params_test['steering_angle'] = 0
    params_test['closest_waypoints'] = (0, 1)
    params_test['x'] = params_test['waypoints'][params_test['closest_waypoints'][0]][0]
    params_test['y'] = params_test['waypoints'][params_test['closest_waypoints'][0]][1]

    # Center line - in corridor (left and right)
    re = RewardEvaluator(params_test)
    assert re.is_in_optimized_corridor()
    params_test['is_left_of_center'] = False
    re = RewardEvaluator(params_test)
    assert re.is_in_optimized_corridor()

    # Center line - out of corridor (left and right)
    params_test['distance_from_center'] = CENTERLINE_FOLLOW_RATIO_TRESHOLD * 2.2 * re.track_width
    params_test['is_left_of_center'] = True
    re = RewardEvaluator(params_test)
    assert not re.is_in_optimized_corridor()
    params_test['is_left_of_center'] = False
    re = RewardEvaluator(params_test)
    assert not re.is_in_optimized_corridor()

    # BEFORE TURN LEFT  - in corridor more right
    params_test['closest_waypoints'] = (8, 9)
    params_test['distance_from_center'] = CENTERLINE_FOLLOW_RATIO_TRESHOLD * 2.2 * re.track_width
    params_test['is_left_of_center'] = True
    re = RewardEvaluator(params_test)
    assert not re.is_in_optimized_corridor()
    params_test['is_left_of_center'] = False
    re = RewardEvaluator(params_test)
    assert not re.is_in_optimized_corridor()

    params_test['distance_from_center'] = CENTERLINE_FOLLOW_RATIO_TRESHOLD * 0.4 * re.track_width
    params_test['is_left_of_center'] = True
    re = RewardEvaluator(params_test)
    assert re.is_in_optimized_corridor()
    params_test['is_left_of_center'] = False
    re = RewardEvaluator(params_test)
    assert re.is_in_optimized_corridor()

    params_test['distance_from_center'] = CENTERLINE_FOLLOW_RATIO_TRESHOLD * 0.8 * re.track_width
    params_test['is_left_of_center'] = True
    re = RewardEvaluator(params_test)
    assert not re.is_in_optimized_corridor()
    params_test['is_left_of_center'] = False
    re = RewardEvaluator(params_test)
    assert re.is_in_optimized_corridor()

    # TEST IN CURVE - vnitrni strana
    params_test['closest_waypoints'] = (15, 16)
    params_test['distance_from_center'] = CENTERLINE_FOLLOW_RATIO_TRESHOLD * 2.2 * re.track_width
    params_test['is_left_of_center'] = True
    re = RewardEvaluator(params_test)
    assert not re.is_in_optimized_corridor()
    params_test['is_left_of_center'] = False
    re = RewardEvaluator(params_test)
    assert not re.is_in_optimized_corridor()

    params_test['distance_from_center'] = CENTERLINE_FOLLOW_RATIO_TRESHOLD * 0.4 * re.track_width
    params_test['is_left_of_center'] = True
    re = RewardEvaluator(params_test)
    assert re.is_in_optimized_corridor()
    params_test['is_left_of_center'] = False
    re = RewardEvaluator(params_test)
    assert re.is_in_optimized_corridor()

    # Prujezd zatacka vnitrni strana
    params_test['distance_from_center'] = CENTERLINE_FOLLOW_RATIO_TRESHOLD * 0.8 * re.track_width
    params_test['is_left_of_center'] = True
    re = RewardEvaluator(params_test)
    assert re.is_in_optimized_corridor()
    params_test['is_left_of_center'] = False
    re = RewardEvaluator(params_test)
    assert not re.is_in_optimized_corridor()


def print_get_optimum_speed_ratio(test_params):
    params_test = test_params()
    params_test['distance_from_center'] = 0
    params_test['steering_angle'] = 0
    ind = 0
    for w in params_test['waypoints']:
        params_test['closest_waypoints'][0] = ind
        params_test['closest_waypoints'][1] = ind + 1
        params_test['x'] = w[0]
        params_test['y'] = w[1]
        re = RewardEvaluator(params_test)
        re.heading = re.get_heading_between_waypoints(w, re.get_way_point(ind + 1))
        print(str(ind) + " speed ratio : " + str(re.get_optimum_speed_ratio()))
        ind = ind + 1
    print(" ")


def print_is_in_turn():
    params_test = get_test_params()
    params_test['distance_from_center'] = 0
    params_test['steering_angle'] = 0
    for ind in range(len(params_test['waypoints'])):
        params_test['closest_waypoints'][0] = ind
        params_test['closest_waypoints'][1] = ind + 1
        re = RewardEvaluator(params_test)
        print(str(ind) + " is_in_turn : " + str(re.is_in_turn()))
    print(" ")


@pytest.mark.parametrize("track_name", track_names)
def test_is_in_turn(test_params, track_name):
    params_test = test_params(track_name)
    params_test['heading'] = 0
    params_test['waypoints'] = [(0, 0), (1, 0), (2, 0), (3, 1), (4, 1), (5, 1), (6, -6), (-1, -6), (-1, 0)]
    params_test['closest_waypoints'] = (0, 1)
    re = RewardEvaluator(params_test)
    assert not re.is_in_turn()
    params_test['closest_waypoints'] = (1, 2)
    re = RewardEvaluator(params_test)
    assert not re.is_in_turn()
    params_test['closest_waypoints'] = (2, 3)
    re = RewardEvaluator(params_test)
    assert re.is_in_turn()
    params_test['closest_waypoints'] = (5, 6)
    re = RewardEvaluator(params_test)
    assert re.is_in_turn()


def print_get_turn_angle():
    params_test = get_test_params()
    params_test['distance_from_center'] = 0
    params_test['steering_angle'] = 0
    for ind in range(len(params_test['waypoints'])):
        params_test['closest_waypoints'][0] = ind
        params_test['closest_waypoints'][1] = ind + 1
        re = RewardEvaluator(params_test)
        print(str(ind) + " get_turn_angle : {0:.1f}".format(re.get_turn_angle()))
    print(" ")


def print_get_expected_turn_direction():
    params_test = get_test_params()
    params_test['distance_from_center'] = 0
    params_test['steering_angle'] = 0
    for ind in range(len(params_test['waypoints'])):
        params_test['closest_waypoints'][0] = ind
        params_test['closest_waypoints'][1] = ind + 1
        re = RewardEvaluator(params_test)
        print(str(ind) + " getCurveDirectio : " + re.get_expected_turn_direction())
    print(" ")


@pytest.mark.parametrize("track_name", track_names)
def test_get_turn_angle(test_params, track_name):
    params_test = test_params(track_name)
    params_test['heading'] = 0
    params_test['waypoints'] = [(0, 0), (1, 0), (2, 0), (3, 1), (4, 1), (5, 1), (6, -6), (-1, -6), (-1, 0)]
    params_test['closest_waypoints'] = (0, 1)
    re = RewardEvaluator(params_test)
    assert re.get_turn_angle() == 0
    params_test['closest_waypoints'] = (1, 2)
    re = RewardEvaluator(params_test)
    assert re.get_turn_angle() == 0
    params_test['closest_waypoints'] = (2, 3)
    re = RewardEvaluator(params_test)
    assert re.get_turn_angle() == 45
    params_test['closest_waypoints'] = (5, 6)
    re = RewardEvaluator(params_test)
    assert re.get_turn_angle() == -81.86989764584403


@pytest.mark.parametrize("track_name", track_names)
def test_reached_target(test_params, track_name):
    params_test = test_params(track_name)
    max_way_point_index = len(params_test['waypoints']) - 1
    params_test['closest_waypoints'] = (max_way_point_index - 1, max_way_point_index)
    re = RewardEvaluator(params_test)
    assert re.reached_target()
    max_way_point_index = len(params_test['waypoints']) - 5
    params_test['closest_waypoints'] = (max_way_point_index - 1, max_way_point_index)
    re = RewardEvaluator(params_test)
    assert not re.reached_target()


@pytest.mark.parametrize("track_name", track_names)
def test_evaluation(test_params, track_name):
    params_test = get_test_params(track_name)
    params_test['heading'] = 0
    params_test['track_width'] = 10
    params_test['distance_from_center'] = 0
    params_test['is_left_of_center'] = True
    params_test['steering_angle'] = 0
    params_test['closest_waypoints'] = (0, 1)
    params_test['speed'] = 3
    params_test['x'] = params_test['waypoints'][params_test['closest_waypoints'][0]][0]
    params_test['y'] = params_test['waypoints'][params_test['closest_waypoints'][0]][1]
    re = RewardEvaluator(params_test)
    re.evaluate()

    params_test = get_test_params()
    params_test['heading'] = 0
    params_test['track_width'] = 10
    params_test['distance_from_center'] = 0
    params_test['is_left_of_center'] = True
    params_test['steering_angle'] = 0
    params_test['closest_waypoints'] = (69, 70)
    params_test['speed'] = 3
    params_test['x'] = params_test['waypoints'][params_test['closest_waypoints'][0]][0]
    params_test['y'] = params_test['waypoints'][params_test['closest_waypoints'][0]][1]
    re = RewardEvaluator(params_test)
    re.evaluate()

    # print_is_in_turn()
    # print_get_turn_angle()
    # print_get_expected_turn_direction()
