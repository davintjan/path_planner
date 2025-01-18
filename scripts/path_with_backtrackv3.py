#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import cv2
import serial
import time

class DubinsPathPlanner:
    def __init__(self, start, turn_radius=1, travel_distance=1, straight_step_distance = 50, goal_threshold = 50):
        self.start = np.array(start)
        self.path = [self.start]
        self.path_strings = []
        self.angle = 0 # Initial angle in radians (relative to the y-axis)
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
        self.red_ranges = [
        ([0, 100, 100], [10, 255, 255]),
        ([160, 100, 100], [180, 255, 255]),
        ([10, 80, 80], [20, 255, 255]),
        ([140, 50, 50], [160, 255, 255]),
        ([170, 50, 50], [180, 255, 200]),
        ([5, 70, 50], [15, 255, 150]),
        ([160, 50, 50], [170, 200, 200]),
        ([0, 80, 50], [10, 200, 150]),  
        ]
        self.green_ranges = [
            ([35, 50, 50], [85, 255, 255]),
            ([35, 80, 80], [50, 255, 255]),
            ([30, 100, 100], [40, 255, 255]),
        ]
        self.pink_range = ([150, 50, 50], [170, 255, 255])
        self.blue_range = [([102, 87, 44], [115, 255, 92]),
                           ([102, 87, 121], [115, 255, 147])]
        # self.pink_range = ([108, 160, 120], [115, 235, 175])
        # self.blue_range = [([4, 200, 150], [7, 220, 190])]
        self.init_tracker = True
    def setup_serial(self): # Call this to establish connection to arduino
        self.arduino = serial.Serial(port='/dev/cu.usbmodem11201', baudrate=115200, timeout=2)

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
            new_y_position = self.path[-1][0] - distance
            new_position = np.array([new_y_position, self.path[-1][1]])
            # if self.arduino:
            #     self.arduino.write(b'f')  # Send the byte 'f' to Arduino, for future reference, you can send 'u' to turn right, 'i' to turn left
            #     time.sleep(0.05) # Ensure you pause half a second to let f truly runs
            #     # You can make this as loop (change distance into a number of f (one f is 0.5 mm), then loop over)
            #     print("Sent 'f' to Arduino")
        else:
            dy = np.cos(np.deg2rad(60)) #hard coded for 30 deg
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
            dy = self.travel_distance * np.sin(self.angle)
            dx = -self.travel_distance * np.cos(self.angle)  # Negative to turn along negative x direction
            new_position = self.path[-1] + np.array([dx, dy])
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
        max_iterations = 5000  # Set a reasonable maximum number of iterations
        with tqdm(total=max_iterations, desc="Generating Path") as pbar:
            while np.linalg.norm(self.path[-1] - self.goal) > self.goal_threshold: #how close to goal
                direction_to_goal = self.goal - self.path[-1]
                angle_to_goal = np.arctan2(direction_to_goal[0], direction_to_goal[1])
                angle_diff = angle_to_goal - self.angle
                if not self.try_turn and np.abs(angle_diff) > np.deg2rad(28) and np.abs(angle_diff) <= np.deg2rad(40):
                    print("trying turn")
                    if not self.turn_made:
                        self.turn_arc(30)
                else:
                    print("move straight")
                    self.move_straight(self.straight_step_distance)
                if self.check_collision(self.path[-1]):
                    self.backtrack()  # Backtrack if a collision is detected

                pbar.update(1)  # Update the progress bar
                if pbar.n >= max_iterations:
                    print("Reached maximum iterations, failed, stopping path generation.")
                    break
        print(self.path)
        print("Path generated, successfully reached goal.")


    def create_masks(self, hsv, color_ranges):
        mask = np.zeros_like(hsv[:, :, 0])
        for (lower, upper) in color_ranges:
            lower_np = np.array(lower)
            upper_np = np.array(upper)
            mask |= cv2.inRange(hsv, lower_np, upper_np)
        return mask

    def process_frame(self, frame):
        self.frame = frame
        frame = np.flipud(frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_red = self.create_masks(hsv, self.red_ranges)
        mask_green = self.create_masks(hsv, self.green_ranges)
        mask_pink = cv2.inRange(hsv, np.array(self.pink_range[0]), np.array(self.pink_range[1]))
        mask_combined = (mask_red | mask_green) & cv2.bitwise_not(mask_pink)

        contours, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
        print("Path commands: ", self.path_strings)

        plt.plot(path[:, 0], path[:, 1], marker='o')
        plt.plot(self.start[0], self.start[1], 'go', label='Start')

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

    def plot_path_updated(self, path):
        path = np.array(path)
        frame = np.flipud(self.frame)
        print("Path commands: ", self.path_strings)

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

    def track_needle_test(self):
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

    def track_needle(self):
        video = cv2.VideoCapture(2)

        width = 640
        height = 480
        video.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        video.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        for i in range(5): #wait for lighting to stabliize if necessary, compute with 5th frame
            ret, frame = video.read()
            if not ret:
                break
        frame = np.flipud(frame) #flip y axis to match rest of code
        # Thresholding
        threshold_ret, threshold_frame = cv2.threshold(frame, 100, 255, cv2.THRESH_TOZERO) #might need to adjust threshold here to track needle
        # plt.imshow(threshold_frame) #VISUALIZE FRAME HERE FOR TUNING
        # plt.show()
        if self.init_tracker:
            self.tracker = cv2.legacy.TrackerCSRT_create()
            bbox = cv2.selectROI("Select needle", threshold_frame, False)
            self.tracker.init(frame, bbox)
            self.init_tracker = False #no need to do this on the next frames
            cv2.destroyAllWindows()

        # Update tracker
        success, bbox = self.tracker.update(threshold_frame)
        if success:
            x, y, w, h = [int(i) for i in bbox]
            cv2.rectangle(threshold_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Extract ROI and calculate moments
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            roi = gray_frame[y:y + h, x:x + w]
            moments = cv2.moments(roi)
            if moments["m00"] != 0:
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
                centroid = (cx + x, cy + y)  # Adjust centroid to the original frame coordinates
                # cv2.circle(frame, centroid, 5, (0, 0, 255), -1)

            # # Display the frame with tracking #uncomment to visualize for debugging
            # cv2.imshow("Tracking", frame)

            # # Exit if 'q' is pressed
            # if cv2.waitKey(40) & 0xFF == ord('q'):
            #     break
        video.release()
        # cv2.destroyAllWindows()
        return centroid

    def write_read(self, x):
        self.arduino.write(bytes(x, 'utf-8'))
        time.sleep(0.05)  # Wait for the Arduino to process the command
        data = self.arduino.readline()
        return data

    def command_arduino_direct(self, command):
        command_arr = command
        print(command_arr)
        for command in command_arr:
            response = self.write_read(command)
            print(f"Sent: {command}, Received: {response.decode('utf-8').strip()}")
            time.sleep(0.1)

    def command_arduino(self, text_file):
        with open(text_file, "r") as f:
            lines = f.readlines()
        command_arr = [line.strip() for line in lines]
        print(command_arr)
        for command in command_arr:
            response = self.write_read(command)
            print(f"Sent: {command}, Received: {response.decode('utf-8').strip()}")
            time.sleep(1)
        # self.arduino.close()

    def execute_planned_path(self, allowed_path_error):
        needle_tip_coords = self.track_needle()  # Select needle ROI to initialize tracker
        print(self.path)
        path_taken = []
        steps_after_turn = 0  # To count steps after the last turn
        turn_occurred = False  # Flag to check if a turn has occurred

        # Initial command sequences
        # i = 13
        # while count in range(i):
             
        step = ['f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 't']
        turn = ['u', 'u', 'u', 'f', 'u', 'u', 'f', 'u', 'u', 'f', 'u', 'f', 'u', 'u', 'f', 'u', 'u', 'f', 'u', 'u', 'f',
                'u', 'u', 'f', 'u', 'u', 'f', 'u', 'u', 'u', 'u', 'f', 'f', 'f', 'f', 'f', 'f', 'f']

        path_copy = self.path.copy()
        for i in range(len(self.path_strings)):
            command = self.path_strings[i]
            print('path string')
            print(self.path_strings)
            if command == "step":
                print('actual needle: ', needle_tip_coords)
                print('self path 0 0: ', self.path[0][0])
                while (needle_tip_coords[0] >= self.path[i][0]):
                    print('Before step')
                    print('Current tip coord: ', needle_tip_coords)
                    print('Goal coord: ', self.path[i])
                    # TODO publish to rostopic instead of ffff
                    self.command_arduino_direct('f')
                    needle_tip_coords = self.track_needle()  # Get updated needle coordinates
                    # time.sleep(0.3)  # Pause to allow needle to move
                    print('After step')
                    print('Current tip coord: ', needle_tip_coords)
                    print('Goal coord: ', self.path[i])
                
                print('self path 0 before ')
                print(self.path[0])
                print(self.path[1])
                print('start before ')
                print(self.start)
                path_copy.pop(0)
                self.start = np.array(path_copy[0])
                print('self path 0 after ')
                print(self.path[0])
                print(self.path[1])
                print('start before ')
                print(self.start)
                
                self.initial_obstacle_goal_detection()
                self.plot_path_updated(path_copy)

                self.command_arduino_direct('t')
                path_taken.append("step")

            elif command == "turn":
                self.command_arduino_direct(turn)
                path_taken.append("turn")
                turn_occurred = True  # A turn has occurred
                steps_after_turn = 0  # Reset steps after the turn

            print(path_taken)
            needle_tip_coords = self.track_needle()  # Get updated needle coordinates
            print(needle_tip_coords, 'tip coord')
            print(self.path[i], 'actual coord')

            time.sleep(1)  # Pause to allow needle to move

        # After completing all steps and turns, check if the needle tip is off the path
        if np.linalg.norm(self.path[-1] - needle_tip_coords) > allowed_path_error:
            print("Needle tip is off path, attempting to backtrack...")

            # Initialize the adjustment counter
            adjustment_counter = 1
            sign = 1  # Start by adding 'u'

            while True:
                # Retract steps after the turn
                for _ in range(steps_after_turn):
                    self.command_arduino("step_back.txt")  # Retract steps
                # Undo the turn
                self.command_arduino("turn_back.txt")
                print("Backtrack complete. Adjusting turn...")

                # Adjust the turn: alternate between adding and removing 'u'
                if sign > 0:
                    turn.extend(['u'] * adjustment_counter)  # Add 'u'
                    print(f"Adding {adjustment_counter} 'u's. Turn command: {turn}")
                else:
                    turn = turn[:-adjustment_counter]  # Remove 'u'
                    print(f"Removing {adjustment_counter} 'u's. Turn command: {turn}")

                # Perform the turn adjustment and reinsertion
                self.command_arduino_direct(turn)

                # Send the step commands again after turn adjustment
                for _ in range(steps_after_turn):
                    self.command_arduino_direct(step)

                # Check the needle's position after re-execution
                needle_tip_coords = self.track_needle()
                print(f"New tip coord after finer adjustment: {needle_tip_coords}")

                # If still off the path, retract and adjust the turn again
                if np.linalg.norm(self.path[-1] - needle_tip_coords) > allowed_path_error:
                    print("Still off the path, retracting and adjusting the turn...")
                    # Toggle between adding and removing
                    sign *= -1
                    adjustment_counter += 1  # Increment adjustment by 1 for next loop
                else:
                    print('Needle reached the goal after fine adjustments.')
                    break  # Exit the loop when the needle reaches the goal

        else:
            print('Path executed successfully, needle reached the goal')

        self.arduino.close()


def main():
    start = (504, 223) #where the needle enters the frame (need to calibrate)
    allowed_path_error = 50 #allowed needle tip error (need to calibrate)
    turn_radius = 8 #turn radius for needle path planning, try to match this with the real needle *in pixel space* (need to calibrate)
    travel_distance = 3 #another turning parameter for needle path planning, try to match this with real needle *in pixel space* (need to calibrate)
    straight_step_distance = 20 #distance to move straight in pixel space (need to calibrate)
    goal_threshold = 0 #the distance the needle needs to be near the goal to be considered successful (maybe need to calibrate)
    planner = DubinsPathPlanner(start, turn_radius, travel_distance, straight_step_distance, goal_threshold)
    planner.initial_obstacle_goal_detection()
    planner.generate_path()
    planner.plot_path()

    print(planner.path_strings)
    planner.setup_serial() # set up serial here
    print("start executing")
    planner.execute_planned_path(allowed_path_error) #uncomment to for integration with jig

if __name__ == "__main__":
    main()
