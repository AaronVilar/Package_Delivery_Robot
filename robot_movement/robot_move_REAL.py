import numpy as np
import math
import matplotlib.pyplot as plt
from PIL import Image

# Parameters
k_Lf = 0.1  # look forward gain
Lfc = 10.0  # [cm] look-forward constant
dt = 0.025  # [s] time tick
set_point = 0.0  # This is the set point in motor speed (hz)

show_animation = False


class Robot:
    """Creates a robot class that defines physical dimensions"""

    def __init__(self, pose=[0.0, 0.0, 0.0], velocity=0.0, angular_velocity=0.0, wheel_speed=[0.0, 0.0]):
        self.wheel_radius = 3.5  # cm
        self.wheel_span = 18.56  # cm
        self.max_encoder_speed = 1150  # hz
        self.gear_ratio = 74.83  # 74.83:1
        self.max_wheel_speed = (2 * math.pi * self.max_encoder_speed) / (self.gear_ratio * 12.0)  # rad/s
        self.max_velocity = self.max_wheel_speed * self.wheel_radius  # cm/s
        self.max_angular_velocity = 2 * self.max_velocity / self.wheel_span  # rad/s
        self.max_acceleration = self.max_velocity / 1.0  # cm/s^2

        self.pose = pose  # [x [cm], y [cm], theta [radians]]
        self.velocity = velocity  # cm/s
        self.angular_velocity = angular_velocity  # rad/s
        self.wheel_speed = wheel_speed  # rad/s
        self.encoder_speed = [0.0, 0.0]

        self.velocity_history = [velocity]
        self.left_wheel_history = [wheel_speed[0]]
        self.right_wheel_history = [wheel_speed[1]]

    def calc_distance(self, x, y):
        dx = self.pose[0] - x
        dy = self.pose[1] - y
        return math.hypot(dx, dy)

    def rotate_to(self, x, y):
        self.pose[2] = math.atan2((y - self.pose[1]), (x - self.pose[0])) - 0.01 # 0.57 degree offset

    def control_motors(self, turn_radius):
        # Calculate velocity and angular velocity
        print(self.velocity)
        np.append(self.velocity_history, [self.velocity], axis=0)
        self.angular_velocity = self.velocity / turn_radius

        # Calculate motor commands
        left_velocity = self.angular_velocity * (turn_radius - (self.wheel_span / 2))
        right_velocity = self.angular_velocity * (turn_radius + (self.wheel_span / 2))

        self.wheel_speed[0] = left_velocity / self.wheel_radius
        self.left_wheel_history = np.append(self.left_wheel_history, [self.wheel_speed[0]], axis=0)
        self.wheel_speed[1] = right_velocity / self.wheel_radius
        self.right_wheel_history = np.append(self.right_wheel_history, [self.wheel_speed[1]], axis=0)

        self.encoder_speed[0] = (self.gear_ratio * 12.0 * self.wheel_speed[0]) / (2.0 * math.pi)
        self.encoder_speed[1] = (self.gear_ratio * 12.0 * self.wheel_speed[1]) / (2.0 * math.pi)

    def read_encoders(self):
        return

    def update_pose(self): #####################
        self.pose[0] += self.velocity * math.cos(self.pose[2]) * dt
        self.pose[1] += self.velocity * math.sin(self.pose[2]) * dt
        self.pose[2] += self.angular_velocity * dt
        return


class AStarPlanner:
    def __init__(self, obstacles_x, obstacles_y, resolution, robot_radius):
        self.resolution = resolution
        self.rr = robot_radius
        self.min_x, self.min_y = 0, 0
        self.max_x, self. max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(obstacles_x, obstacles_y)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x), self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            # show graph
            if False:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                            self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(
                                                    0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                # print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
            # generate final course
            rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
                self.calc_grid_position(goal_node.y, self.min_y)]
            parent_index = goal_node.parent_index
            while parent_index != -1:
                n = closed_set[parent_index]
                rx.append(self.calc_grid_position(n.x, self.min_x))
                ry.append(self.calc_grid_position(n.y, self.min_y))
                parent_index = n.parent_index

            return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
            w = 1.0  # weight of heuristic
            d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
            return d

    def calc_grid_position(self, index, min_position):
            """
            calc grid position

            :param index:
            :param min_position:
            :return:
            """
            pos = index * self.resolution + min_position
            return pos

    def calc_xy_index(self, position, min_pos):
            return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
            return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
            px = self.calc_grid_position(node.x, self.min_x)
            py = self.calc_grid_position(node.y, self.min_y)

            if px < self.min_x:
                return False
            elif py < self.min_y:
                return False
            elif px >= self.max_x:
                return False
            elif py >= self.max_y:
                return False

            # collision check
            if self.obstacle_map[node.x][node.y]:
                return False

            return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
            # dx, dy, cost
            motion = [[1, 0, 1],
                      [0, 1, 1],
                      [-1, 0, 1],
                      [0, -1, 1],
                      [-1, -1, math.sqrt(2)],
                      [-1, 1, math.sqrt(2)],
                      [1, -1, math.sqrt(2)],
                      [1, 1, math.sqrt(2)]]

            return motion


class Map:

    def __init__(self):
        self.obstacles_x = []
        self.obstacles_y = []
        self.start_x = 43.0
        self.start_y = 206.0
        self.goal_x = 70.0
        self.goal_y = 60.0

    def generate_map(self):
        # for i in range(0, 60):
        #     self.obstacles_x.append(i)
        #     self.obstacles_y.append(0)
        # for i in range(0, 60):
        #     self.obstacles_x.append(60.0)
        #     self.obstacles_y.append(i)
        # for i in range(0, 61):
        #     self.obstacles_x.append(i)
        #     self.obstacles_y.append(60.0)
        # for i in range(0, 61):
        #     self.obstacles_x.append(0)
        #     self.obstacles_y.append(i)
        # for i in range(0, 40):
        #     self.obstacles_x.append(20.0)
        #     self.obstacles_y.append(i)
        # for i in range(0, 40):
        #     self.obstacles_x.append(40.0)
        #     self.obstacles_y.append(60.0 - i)

        map_image = Image.open(r"C:\Users\aaron\PycharmProjects\Robot_Movement\THE_MAP.png")
        map_array = np.asarray(map_image)[:, :, 2]
        y, x = np.shape(map_array)
        for i in range(x):
            for j in range(y):
                if map_array[j, i] == 0:
                    self.obstacles_x.append(i)
                    self.obstacles_y.append(j)

    def plot_map(self):
        plt.plot(self.obstacles_x, self.obstacles_y, ".k")
        plt.plot(self.start_x, self.start_y, "og")
        plt.plot(self.goal_x, self.goal_y, "xb")
        plt.grid(True)
        plt.axis("equal")


class Path:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        x_diff = np.subtract(x[1:len(x)], x[0:len(x)-1])
        y_diff = np.subtract(y[1:len(y)], y[0:len(y)-1])
        self.path_length = np.sum(np.hypot(x_diff, y_diff))
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


class PathTaken:

    def __init__(self):
        self.x_pose = []
        self.y_pose = []
        self.velocity = []
        self.time = []

    def append(self, robot, time):
        self.x_pose.append(robot.pose[0])
        self.y_pose.append(robot.pose[1])
        self.velocity.append(robot.velocity)
        self.time.append(time)


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

    alpha = math.atan2(path_y - robot.pose[1], path_x - robot.pose[0]) - robot.pose[2]  # radians
    turn_radius = look_forward_dist / (2 * math.sin(alpha))
    robot.angular_velocity = 2 * robot.velocity * math.sin(alpha) / look_forward_dist  # rad/s

    return alpha, turn_radius, index


def main():
    # Create map
    print("create map")
    map = Map()
    map.generate_map()
    grid_size = 1.0  # cm
    robot_radius = 10.0  # cm

    # Create a path
    print("create a path")
    a_star = AStarPlanner(map.obstacles_x, map.obstacles_y, grid_size, robot_radius)
    print("plan")
    path_x, path_y = a_star.planning(map.start_x, map.start_y, map.goal_x, map.goal_y)

    path_x = path_x[::-1]
    path_y = path_y[::-1]
    path = Path(path_x, path_y)

    # Create Robot at starting position with 0 velocity
    print("Create Robot at starting position with 0 velocity")
    robot = Robot(pose=[map.start_x, map.start_y, 0])

    # Rotate robot towards first point
    print("Rotate robot towards first point")
    robot.rotate_to(path.x[1], path.y[1])

    last_index = len(path_x) - 1
    current_time = 0.0
    simulation_time = 100
    target_index, _ = path.search_path_index(robot)

    path_taken = PathTaken()
    path_taken.append(robot, current_time)
    print("simulate")
    while simulation_time >= current_time and last_index > target_index:
        robot.velocity += (robot.max_acceleration * dt)
        if robot.velocity > robot.max_velocity:
            robot.velocity = robot.max_velocity

        _, turn_radius, target_index = pure_pursuit_steer_control(robot, path, target_index)

        robot.control_motors(turn_radius)
        robot.update_pose()

        current_time += dt
        path_taken.append(robot, current_time)

        if show_animation:
            plt.cla()
            # For stopping simulation with the esc key
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None]
            )
            map.plot_map()
            plt.plot(path_x, path_y, "-r", label="course")
            plt.plot(path_taken.x_pose, path_taken.y_pose, "-b", label="trajectory")
            plt.plot(path_x[target_index], path_y[target_index], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(robot.velocity * 3.6)[:4])
            plt.pause(0.001)

    assert last_index >= target_index
    if show_animation:  # pragma: no cover
        plt.subplots(1)
        plt.plot(path_taken.time, robot.left_wheel_history, "-r", label="left wheel speed")
        plt.plot(path_taken.time, robot.right_wheel_history, "-b", label="right wheel speed")
        plt.legend()
        plt.axis("tight")
        plt.grid(True)
        plt.show()

    # plt.show()


if __name__ == '__main__':
    main()
