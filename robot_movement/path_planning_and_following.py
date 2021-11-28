import numpy as np
import math
import matplotlib.pyplot as plt

# Parameters
k_Lf = 0.1  # look forward gain
Lfc = 2  # [m] look-forward constant
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle


class Robot:
    """Creates a robot class that defines physical dimensions"""

    def __init__(self, wheel_radius=0.035, wheel_span=0.02,
                 pose=[0.0, 0.0, 0.0], velocity=0.0, angular_velocity=0.0, wheel_speed=[0.0, 0.0]):
        self.wheel_radius = wheel_radius  # measured in meters
        self.wheel_span = wheel_span  # measured in meters
        self.pose = pose  # [x, y, theta]
        self.velocity = velocity
        self.angular_velocity = angular_velocity
        self.wheel_speed = wheel_speed

    def calc_distance(self, x, y):
        dx = self.pose[0] - x
        dy = self.pose[1] - y
        return math.hypot(dx, dy)

    def control_motors(self, controller_out, alpha, turn_radius):
        left_velocity = (2 * self.velocity - self.angular_velocity * self.wheel_span) / (2 * self.wheel_radius)
        right_velocity = (2 * self.velocity + self.angular_velocity * self.wheel_span) / (2 * self.wheel_radius)

        left_angular_v = left_velocity / self.wheel_radius
        right_angular_v = right_velocity / self.wheel_radius

    def read_encoders(self):


def wheel_controller(set_point, current):
    return Kp * (set_point - current)


class Path:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.old_nearest_point_index = None

    def search_path_index(self, robot):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [robot.pose[0] - ix for ix in self.x]
            dy = [robot.pose[1] - iy for iy in self.y]
            d = np.hypot(dx, dy)
            index = np.argmin(d)
            self.old_nearest_point_index = index
        else:
            index = self.old_nearest_point_index
            distance_this_index = robot.calc_distance(self.x[index],
                                                      self.y[index])
            while True:
                distance_next_index = robot.calc_distance(self.x[index + 1],
                                                          self.y[index + 1])
                if distance_this_index < distance_next_index:
                    break
                index = index + 1 if (index + 1) < len(self.x) else index
                distance_this_index = distance_next_index
            self.old_nearest_point_index = index

        look_forward_dist = k_Lf * robot.velocity + Lfc  # update look forward distance

        # search look ahead target point index
        while look_forward_dist > robot.calc_distance(self.x[index], self.y[index]):
            if (index + 1) >= len(self.x):
                break  # not exceed goal
            index += 1

        return index, look_forward_dist


def pure_pursuit_steer_control(robot, path, target_index):
    index, look_forward_dist = path.search_path_index(robot)

    if target_index >= index:
        index = target_index

    if index < len(path.x):
        path_x = path.x[index]
        path_y = path.y[index]
    else:  # toward goal
        path_x = path.x[-1]
        path_y = path.y[-1]
        index = len(path.x) - 1

    alpha = math.atan2(path_x - robot.pose[1], path_x - robot.pose[0]) - robot.pose[2]
    turn_radius = look_forward_dist / (2 * math.sin(alpha))

    return alpha, turn_radius, index


def main():
    # Create a path
    path_x = np.arange(0, 50, 0.5)
    path_y = [math.sin(ix / 5.0) * ix / 2.0 for ix in path_x]
    path = Path(path_x, path_y)

    # Create a set point for PID
    set_point = 1.0  # This is the set point in m/s

    # Create Robot at 0,0,0 with 0 velocity
    robot = Robot()
    last_index = len(path_x) - 1
    current_time = 0.0
    simulation_time = 100.0
    target_index, _ = path.search_path_index(robot)

    while simulation_time >= current_time and last_index > target_index:
        # Calculate contol input
        controller_out = wheel_controller(set_point, robot.velocity)
        alpha, turn_radius, target_index = pure_pursuit_steer_control(robot,
                                                                      path, target_index)

        robot.control_motors(controller_out, alpha, turn_radius)
        robot.read_encoders()

        current_time += dt


if __name__ == '__main__':
    main()
