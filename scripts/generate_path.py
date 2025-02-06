#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import cv2
import serial
import time

""" THIS FILE IS PURELY USE TO GENERATE THE PATH AND GET THE COORDINATE OF EACH NODE """


class DubinsPathPlanner:
    def __init__(self, start, turn_radius=1, travel_distance=1, straight_step_distance = 20, goal_threshold = 5):
        self.start = np.array(start)
        self.path = [self.start]
        self.path_strings = []
        self.angle = 0 # Initial angle in radians (relative to the x-axis)
        self.obstacles = []
        self.turn_radius = turn_radius  # Minimum turn radius
        self.travel_distance = travel_distance  # Minimum turn radius
        self.straight_step_distance = straight_step_distance
        self.turn_made = False  # Flag to track if a turn has been made
        self.position_stack = []  # Stack to store previous positions for backtracking
        self.try_turn = False
        self.init_tracker = False
        self.goal_threshold = goal_threshold
        # Color thresholding
        self.webcam_device_index = 0
        # self.min_contour_area = 600
        self.min_contour_area = 30
        # OBS
        self.red_ranges = [
        ([2, 225, 150], [8, 260, 170]),
        ]
        # GOAL
        self.blue_range = [([102, 87, 44], [115, 255, 92]),
                           ([102, 87, 121], [115, 255, 147])]
        # self.pink_range = ([108, 160, 120], [115, 235, 175])
        # self.blue_range = [([4, 200, 150], [7, 220, 190])]
        self.init_tracker = True

    def initial_obstacle_goal_detection(self):
        video = cv2.VideoCapture(2)

        width = 640
        height = 480
        video.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        video.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not video.isOpened():
            print("Error: Could not open webcam.")
            exit()

        for _ in range(10): #cycle through 10 frames and process the 10th frame
            ret, frame = video.read()
            if not ret:
                print("Error: No frame to read")
                break
        print('video read successfully')
        obstacle_contours, goal_contour = self.process_frame(frame)
        # import ipdb; ipdb.set_trace()
        # add contours to obstacle map
        for contour in obstacle_contours:
            x, y, w, h = cv2.boundingRect(contour)
            # print(x, y, w, h)
            # obstacle_margin = 5
            # x -= obstacle_margin
            # y -= obstacle_margin
            # w += obstacle_margin
            # h += obstacle_margin
            obstacle = {'x_min': x, 'x_max': x + w, 'y_min': y, 'y_max': y + h}
            self.obstacles.append(obstacle)
        for contour in goal_contour:
            x, y, w, h = cv2.boundingRect(contour)
            # goal_rect = {'x_min': x, 'x_max': x + w, 'y_min': y, 'y_max': y + h}
            # self.goal_rect.append(goal_rect) #for debugging
            self.goal = (x + w / 2, y + h / 2)
        for obstacle in self.obstacles:
            rect = plt.Rectangle((obstacle['x_min'], obstacle['y_min']),
                                 obstacle['x_max'] - obstacle['x_min'],
                                 obstacle['y_max'] - obstacle['y_min'],
                                 color='gray', alpha=0.5, label = "Obstacle")
        circle_center = (self.goal[0], self.goal[1])
        circle_radius = 0.5  # Example radius of the circle

    def move_straight(self, distance):
        if not self.turn_made:
            # import ipdb; ipdb.set_trace()
            new_y_position = self.path[-1][1] - distance
            new_x_position = self.path[-1][0]
            new_position = np.array([new_x_position, new_y_position])
            # if self.arduino:
            #     self.arduino.write(b'f')  # Send the byte 'f' to Arduino, for future reference, you can send 'u' to turn right, 'i' to turn left
            #     time.sleep(0.05) # Ensure you pause half a second to let f truly runs
            #     # You can make this as loop (change distance into a number of f (one f is 0.5 mm), then loop over)
            #     print("Sent 'f' to Arduino")
        else:
            dy = -np.cos(np.deg2rad(60)) #hard coded for 30 deg
            dx = -np.sin(np.deg2rad(60))
            new_position = self.path[-1] + distance * np.array([dx, dy])

        if not self.check_collision_along_path(self.path[-1], new_position):
            self.position_stack.append(self.path[-1])  # Push current position to stack
            self.path.append(new_position)
            self.path_strings.append("step")
        else:
            self.backtrack()
            print("check straight collision")
        self.try_turn = False

    def turn_arc(self, angle_deg):
        angle_rad = np.deg2rad(angle_deg)
        arc_length = self.turn_radius * angle_rad
        steps = int(arc_length)
        if steps == 0:
            steps = 1
        angle_step = angle_rad / steps
        for i in range(steps):
            self.angle += angle_step if angle_deg > 0 else -angle_step
            dy = self.travel_distance * np.cos(self.angle)
            dx = self.travel_distance * np.sin(self.angle)  # Negative to turn along negative x direction
            new_position = self.path[-1] - np.array([dx, dy])
            if not self.check_collision_along_path(self.path[-1], new_position):
                self.position_stack.append(self.path[-1])  # Push current position to stack
                self.path.append(new_position)
                if i == 0:
                    self.path_strings.append("turn")
                self.turn_made = True
                self.try_turn = True
            else:
                for j in range(i):
                    self.backtrack()
                self.try_turn = True
                break

    def check_collision(self, point):
        for obstacle in self.obstacles:
            if point[0] >= obstacle["x_min"] and point[0] <= obstacle["x_max"]:
                if point[1] >= obstacle["y_min"] and point[1] <= obstacle["y_max"]:
                    return True
        return False

    def check_collision_along_path(self, start_point, end_point, num_steps=10):
        start_point = np.array(start_point)  # Ensure start_point is an array
        end_point = np.array(end_point)
        for i in range(num_steps + 1):
            intermediate_point = start_point + i / num_steps * (end_point - start_point)
            if self.check_collision(intermediate_point):
                return True
        return False

    def backtrack(self):
        if self.position_stack:
            self.path_strings.pop()
            self.path.pop()  # Remove the current position
            self.turn_made = False  # Allow turning again

    def generate_path(self):
        max_iterations = 500  # Set a reasonable maximum number of iterations
        with tqdm(total=max_iterations, desc="Generating Path") as pbar:
            while np.linalg.norm(self.path[-1] - self.goal) > self.goal_threshold: #how close to goal
                direction_to_goal = self.goal - self.path[-1]
                print(direction_to_goal)
                angle_to_goal = np.arctan(direction_to_goal[0]/direction_to_goal[1])
                angle_diff = angle_to_goal - self.angle
                print(np.rad2deg(angle_to_goal))
                print("-------")
                if not self.try_turn and np.abs(angle_diff) > np.deg2rad(60) and np.abs(angle_diff) <= np.deg2rad(75):
                    print("trying turn")
                    if not self.turn_made:
                        self.turn_arc(60)
                else:
                    print("move straight")
                    self.move_straight(self.straight_step_distance)
                if self.check_collision(self.path[-1]):
                    self.backtrack()  # Backtrack if a collision is detected

                pbar.update(1)  # Update the progress bar
                if pbar.n >= max_iterations:
                    print("Reached maximum iterations, failed, stopping path generation.")
                    break
        # print(self.path)
        print("-------------------")
        # print(self.path[-1])
        # print(self.goal)
        # print(np.linalg.norm(self.path[-1] - self.goal))
        print("Path generated, successfully reached goal.")


    def create_masks(self, hsv, color_ranges):
        mask = np.zeros_like(hsv[:, :, 0])
        for (lower, upper) in color_ranges:
            lower_np = np.array(lower)
            upper_np = np.array(upper)
            mask |= cv2.inRange(hsv, lower_np, upper_np)
        return mask

    def process_frame(self, frame):
        # TO DOUBLE CHECK COLOR RANGE ALWAYS USE BGR2HSV
        # plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
        # plt.show()
        self.frame = frame
        frame = np.flipud(frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Create a mask for red only
        lower_bound = np.array(self.red_ranges[0][0])
        upper_bound = np.array(self.red_ranges[0][1])
        mask_obstacle = cv2.inRange(hsv, lower_bound, upper_bound)

        # Find contours in the red mask
        contours, _ = cv2.findContours(mask_obstacle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter out small contours
        filtered_obstacle_contours = [contour for contour in contours if cv2.contourArea(contour) > self.min_contour_area]
        plot_map = np.zeros_like(frame)
        # for contour in filtered_obstacle_contours: #for debugging
        #     cv2.drawContours(plot_map, [contour], -1, (0, 255, 0), -1)
        # plt.imshow(plot_map)
        # plt.show()

        goal_mask = self.create_masks(hsv, self.blue_range)
        contours, _ = cv2.findContours(goal_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_goal_contour = [contour for contour in contours if cv2.contourArea(contour) > self.min_contour_area]
        for contour in filtered_goal_contour: #for debugging
            cv2.drawContours(plot_map, [contour], -1, (0, 255, 0), -1)
        plt.imshow(plot_map)
        plt.show()



        return filtered_obstacle_contours, filtered_goal_contour

    def plot_path(self):
        path = np.array(self.path)
        frame = np.flipud(self.frame)
        # print("Path commands: ", self.path_strings)

        plt.plot(path[:, 0], path[:, 1], marker='o')
        plt.plot(self.start[0], self.start[1], 'go', label='Start')

        # Plot obstacles
        obstacle_margin = 5
        for obstacle in self.obstacles:
            rect = plt.Rectangle(
                (obstacle['x_min'] - obstacle_margin, obstacle['y_min'] - obstacle_margin),
                obstacle['x_max'] - obstacle['x_min'] + 2 * obstacle_margin,
                obstacle['y_max'] - obstacle['y_min'] + 2 * obstacle_margin,
                color='gray', alpha=0.5, label="Obstacle"
            )
            plt.gca().add_patch(rect)
        circle_center = (self.goal[0], self.goal[1])
        circle_radius = 40 # Example radius of the circle
        circle = plt.Circle(circle_center, circle_radius, color='red', fill=True, alpha=0.5, label = 'Goal')
        plt.gca().add_patch(circle)
        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Steerable Needle Path')
        plt.imshow(frame, alpha=.7) #plot original image
        plt.gca().invert_yaxis()
        plt.grid()
        plt.show()

    def plot_path_updated(self, path):
        path = np.array(path)
        frame = np.flipud(self.frame)
        # print("Path commands: ", self.path_strings)

        plt.plot(path[:, 0], path[:, 1], marker='o')
        plt.plot(path[0, 0], path[0, 1], 'go', label='Start')

        # Plot obstacles
        for obstacle in self.obstacles:
            rect = plt.Rectangle((obstacle['x_min'], obstacle['y_min']),
                                 obstacle['x_max'] - obstacle['x_min'],
                                 obstacle['y_max'] - obstacle['y_min'],
                                 color='gray', alpha=0.5, label = "Obstacle")
            plt.gca().add_patch(rect)
        circle_center = (self.goal[0], self.goal[1])
        circle_radius = 50 # Example radius of the circle
        circle = plt.Circle(circle_center, circle_radius, color='red', fill=True, alpha=0.5, label = 'Goal')
        plt.gca().add_patch(circle)
        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Steerable Needle Path')
        plt.imshow(frame, alpha=.7) #plot original image
        plt.gca().invert_yaxis()
        plt.grid()
        plt.show()




def main():
    start = (257, 289) #where the needle enters the frame (need to calibrate)
    allowed_path_error = 50 #allowed needle tip error (need to calibrate)
    turn_radius = 3 #turn radius for needle path planning, try to match this with the real needle *in pixel space* (need to calibrate)
    travel_distance = 15 #another turning parameter for needle path planning, try to match this with real needle *in pixel space* (need to calibrate)
    straight_step_distance = 30 #distance to move straight in pixel space (need to calibrate)
    goal_threshold = 40 #the distance the needle needs to be near the goal to be considered successful (maybe need to calibrate)
    planner = DubinsPathPlanner(start, turn_radius, travel_distance, straight_step_distance, goal_threshold)
    planner.initial_obstacle_goal_detection()
    planner.generate_path()
    planner.plot_path()

    # print(planner.path_strings)

if __name__ == "__main__":
    main()
