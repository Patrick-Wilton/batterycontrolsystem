"""
List of Current Control Settings
* battery, solar and house publish timings vs event based
* control system timing (must be used with publisher timing settings ON)
* real time plotting vs inferred simulation timings
* house and solar input filtering
* battery power output filtering
* ability to display grid power values on plot
* Set variable grid reference point
"""


import zmq
import time
import threading
import numpy as np

import matplotlib.pyplot as plt

from Code.simulation_client import SunSpecDriver


class Subscriber:
    def __init__(self):

        # Settings and Initial Values
        self.battery_read = 0
        self.solar_read = 0
        self.house_read = 0

        self.bat_SOC = 0
        self.solar_power = 0
        self.house_power = 0

        self.solar_int = 0
        self.house_int = 0

        self.lock = threading.Lock()

        self.bat_timing = 5  # minutes
        self.solar_timing = 5  # minutes
        self.house_timing = 5  # minutes

        # Erases Text File Contents
        open("control_power_values.txt", "w+").close()

        # Creates Internal Data Store
        self.data = dict()
        self.data["soc"] = []
        self.data["solar_time"] = []
        self.data["solar_real_time"] = []
        self.data["solar_power"] = []
        self.data["solar_plot"] = []
        self.data["house_time"] = []
        self.data["house_real_time"] = []
        self.data["house_power"] = []
        self.data["house_plot"] = []

        # Sets up Kalman Filters
        self.solar_filtering = True
        self.house_filtering = True
        self.solar_filter = KalmanFilter(1, 0, 1, 0, 1, 0.05, 1)
        self.house_filter = KalmanFilter(1, 0, 1, 700, 1, 0.05, 1)

        # ZeroMQ Subscribing
        bat_port = "8090"
        solar_port = "8091"
        house_port = "8092"
        battery_topic = "0"
        solar_topic = "0"
        house_topic = "0"
        sub_context = zmq.Context()

        self.bat_socket = sub_context.socket(zmq.SUB)
        self.bat_socket.connect("tcp://localhost:%s" % bat_port)
        self.bat_socket.setsockopt_string(zmq.SUBSCRIBE, battery_topic)

        self.solar_socket = sub_context.socket(zmq.SUB)
        self.solar_socket.connect("tcp://localhost:%s" % solar_port)
        self.solar_socket.setsockopt_string(zmq.SUBSCRIBE, solar_topic)

        self.house_socket = sub_context.socket(zmq.SUB)
        self.house_socket.connect("tcp://localhost:%s" % house_port)
        self.house_socket.setsockopt_string(zmq.SUBSCRIBE, house_topic)

        # Plot Timings
        self.bat_hour = 1 / (60 / self.bat_timing)
        self.bat_max_time = int(24 / self.bat_hour)
        self.bat_time_count = 1

        self.solar_hour = 1 / (60 / self.solar_timing)
        self.solar_max_time = int(24 / self.solar_hour)
        self.solar_time_count = 1

        self.house_hour = 1 / (60 / self.house_timing)
        self.house_max_time = int(24 / self.house_hour)
        self.house_time_count = 1

        # Sets Initial Time
        self.initial_time = None

        # Starts Battery Sub Thread
        print('starting battery SOC subscriber')
        self.bat_thread = threading.Thread(target=self.battery_subscriber)
        self.bat_thread.start()

        # Starts Solar Sub Thread
        print('starting solar subscriber')
        self.solar_thread = threading.Thread(target=self.solar_subscriber)
        self.solar_thread.start()

        # Starts House Sub Thread
        print('starting house subscriber')
        self.house_thread = threading.Thread(target=self.house_subscriber)
        self.house_thread.start()

    def battery_subscriber(self):
        while True:
            bat_string = self.bat_socket.recv()
            b_topic, self.bat_SOC = bat_string.split()
            if self.bat_SOC != b'bat_connect':
                self.bat_SOC = int(self.bat_SOC)

                self.write_to_text("SOC", round(time.time() - self.initial_time, 2), self.bat_SOC)
                self.data["soc"].append(self.bat_SOC)

                self.battery_read = 1

    def solar_subscriber(self):
        while True:
            solar_string = self.solar_socket.recv()
            s_topic, self.solar_power = solar_string.split()
            if self.solar_power != b'solar_connect':
                self.solar_power = int(self.solar_power)

                if self.solar_filtering:
                    self.solar_filter.step(0, self.solar_power)
                    self.solar_power = self.solar_filter.current_state()

                self.write_to_text("solar", round(time.time() - self.initial_time, 2), self.solar_power)
                self.data["solar_time"].append(self.solar_time_count / (60 / self.solar_timing))
                self.data["solar_real_time"].append(round(time.time() - self.initial_time, 2) / 3600)
                self.data["solar_power"].append(self.solar_power / 1000)
                if self.data["solar_time"][-1] == 24:
                    self.data["solar_plot"].append(np.nan)
                else:
                    self.data["solar_plot"].append(self.solar_power / 1000)

                if self.solar_time_count == self.solar_max_time:
                    self.solar_time_count = 1
                else:
                    self.solar_time_count += 1
                self.solar_int += 1

                self.solar_read = 1

    def house_subscriber(self):
        while True:
            house_string = self.house_socket.recv()
            h_topic, self.house_power = house_string.split()
            if self.house_power != b'house_connect':
                self.house_power = int(self.house_power)

                if self.house_filtering:
                    self.house_filter.step(0, self.house_power)
                    self.house_power = self.house_filter.current_state()

                self.write_to_text("house", round(time.time() - self.initial_time, 2), self.house_power)
                self.data["house_time"].append(self.house_time_count / (60 / self.house_timing))
                self.data["house_real_time"].append(round(time.time() - self.initial_time, 2) / 3600)
                self.data["house_power"].append(self.house_power / 1000)
                if self.data["house_time"][-1] == 24:
                    self.data["house_plot"].append(np.nan)
                else:
                    self.data["house_plot"].append(self.house_power / 1000)

                if self.house_time_count == self.house_max_time:
                    self.house_time_count = 1
                else:
                    self.house_time_count += 1
                self.house_int += 1

                self.house_read = 1

    def write_to_text(self, device, curr_time, value):
        self.lock.acquire()

        file = open("control_power_values.txt", "a+")

        file.write("\n"+device+" "+str(curr_time)+" "+str(value))
        file.close()

        self.lock.release()


class KalmanFilter:
    def __init__(self, process_dynamics, control_dynamics, measurement_dynamics, current_state_estimate,
                 current_prob_estimate, process_covariance, measurement_covariance):

        self.pro_dyn = process_dynamics
        self.con_dyn = control_dynamics
        self.meas_dyn = measurement_dynamics
        self.curr_state = current_state_estimate
        self.curr_prob = current_prob_estimate
        self.pro_cov = process_covariance
        self.meas_cov = measurement_covariance

    def current_state(self):
        return self.curr_state

    def step(self, control_input, measurement):

        # Prediction Calculations
        predicted_state_estimate = self.pro_dyn * self.curr_state + self.con_dyn * control_input
        predicted_prob_estimate = (self.pro_dyn * self.curr_prob) * self.pro_dyn + self.pro_cov

        # Innovation Calculations
        innovation = measurement - self.meas_dyn * predicted_state_estimate
        innovation_covariance = self.meas_dyn * predicted_prob_estimate * self.meas_dyn + self.meas_cov

        # Posterior Calculations
        kalman_gain = predicted_prob_estimate * self.meas_dyn * 1 / float(innovation_covariance)
        self.curr_state = predicted_state_estimate + kalman_gain * innovation

        # Identity Matrix
        self.curr_prob = (1 - kalman_gain * self.meas_dyn) * predicted_prob_estimate


class Control:
    def __init__(self, sub_time):

        # Initial Values
        self.control_interval = 5  # minutes
        self.sub_time = sub_time
        time_interval_hour = 1 / (60 / self.control_interval)
        self.time_max_count = int(24 / time_interval_hour)
        self.control_time_count = 1
        self.initial_time = None
        self.control_int = 0

        self.bat_power = 0
        self.grid = 0
        self.grid_ref = 0  # Watts
        self.grid_control_dir = "Both"

        # Control Settings
        self.control_time_step = 20  # minutes
        self.control_time = False
        self.bat_filtering = False
        self.real_time_plotting = False
        self.display_grid = True

        # Sets up Kalman Filters
        self.battery_filter = KalmanFilter(1, 0, 1, 0, 1, 0.05, 1)

        # Creates Internal Data Store
        self.control_data = dict()
        self.control_data["bat_power"] = []
        self.control_data["bat_plot"] = []
        self.control_data["grid"] = []
        self.control_data["grid_plot"] = []
        self.control_data["control_time"] = []
        self.control_data["control_real_time"] = []

        # ZeroMQ Publishing
        pub_port = "8093"
        self.pub_topic = 0
        pub_context = zmq.Context()
        self.pub_socket = pub_context.socket(zmq.PUB)
        self.pub_socket.bind("tcp://*:%s" % pub_port)

    def battery_control(self, bat, solar, house, curr_time):

        # Internal Data Store
        self.control_data["control_time"].append((curr_time - 1) / (60 / self.sub_time))
        self.control_data["control_real_time"].append(round(time.time() - self.initial_time, 2) / 3600)
        self.control_data["bat_power"].append(self.bat_power / 1000)
        if abs(self.control_data["control_time"][-1] - 24) < 0.1:
            self.control_data["bat_plot"].append(np.nan)
        else:
            self.control_data["bat_plot"].append(self.bat_power / 1000)

        if self.control_time_count == self.time_max_count:
            self.control_time_count = 1
        else:
            self.control_time_count += 1
        self.control_int += 1

        # Control System
        self.grid = self.bat_power + solar + house
        self.control_data["grid"].append(self.grid / 1000)
        if abs(self.control_data["control_time"][-1] - 24) < 0.1:
            self.control_data["grid_plot"].append(np.nan)
        else:
            self.control_data["grid_plot"].append(self.grid / 1000)

        above_control = self.grid_control_dir == "Above" and self.grid > self.grid_ref
        below_control = self.grid_control_dir == "Below" and self.grid < self.grid_ref
        if above_control or below_control or self.grid_control_dir == "Both":
            self.bat_power = -self.grid + self.bat_power + self.grid_ref
        if (bat == 0 and self.bat_power < 0) or (bat == 100 and self.bat_power > 0):
            self.bat_power = 0
        if self.bat_filtering:
            self.battery_filter.step(0, self.bat_power)
            self.bat_power = self.battery_filter.current_state()

        # Publishing
        self.pub_socket.send_string("%d %d" % (self.pub_topic, self.bat_power))


if __name__ == '__main__':

    # Creates Initial Connect Parameters
    connected = False
    first_read = False
    plot_erase = False
    control_check = False

    house_plot_index = 0
    solar_plot_index = 0
    control_plot_index = 0

    # Starts Subscribers, SunSpec Drivers and Control Class
    sub = Subscriber()
    sun_spec_driver = SunSpecDriver()
    control = Control(sub.solar_timing)

    # Sets Initial Plot Parameters
    plt.figure(figsize=[12, 7])
    plt.axis([0, 24, -6, 8])
    plt.title('One Day')
    plt.xlabel('Time (Hours)')
    plt.ylabel('Power (kW)')
    plt.grid(True)
    ref_line = plt.hlines(control.grid_ref / 1000, 0, 24, linestyles='dashed')
    ref_line.set_label('Reference Grid Power')
    plt.ion()

    house_line, = plt.plot(sub.data["house_time"], sub.data["house_power"], '-o', alpha=0.8, c='b', markersize=2)
    house_line.set_label('House Power')
    solar_line, = plt.plot(sub.data["solar_time"], sub.data["solar_power"], '-o', alpha=0.8, c='r', markersize=2)
    solar_line.set_label('Solar Power')
    battery_line, = plt.plot(control.control_data["control_time"], control.control_data["bat_power"],
                             '-o', alpha=0.8, c='g', markersize=2)
    battery_line.set_label('Battery Power')
    grid_line, = plt.plot(control.control_data["control_time"], control.control_data["grid"],
                          '-o', alpha=0.8, c='m', markersize=2)
    if control.display_grid:
        grid_line.set_label('Grid Power')
    plt.legend()

    house_x = list()
    house_y = list()
    solar_x = list()
    solar_y = list()
    battery_x = list()
    battery_y = list()
    grid_x = list()
    grid_y = list()

    # MAIN LOOP
    print('Starting Control System')
    while True:
        # Boolean Variables
        bat_connect = sub.bat_SOC == b'bat_connect'
        solar_connect = sub.solar_power == b'solar_connect'
        house_connect = sub.house_power == b'house_connect'

        initial_connect = bat_connect and solar_connect and house_connect
        connect_check = bat_connect or solar_connect or house_connect
        all_read = sub.battery_read == 1 and sub.solar_read == 1 and sub.house_read == 1

        # Initial Connection Check
        if all_read:
            first_read = True

        if initial_connect:
            control.pub_socket.send_string("%d %s" % (control.pub_topic, 'connected'))
            sub.initial_time = round(time.time(), 2)
            control.initial_time = sub.initial_time
            connected = True

        # Control Loop
        elif connected and connect_check is False:
            if control.control_time and first_read:
                control_step = sub.solar_time_count % int(control.control_time_step / control.control_interval) == 0
                if control_step and control_check is False:
                    control.battery_control(sub.bat_SOC, sub.solar_power, sub.house_power, sub.solar_time_count)
                    control_check = True
                else:
                    control_check = False

            elif all_read:

                control.battery_control(sub.bat_SOC, sub.solar_power, sub.house_power, sub.solar_time_count)

                sub.battery_read = 0
                sub.solar_read = 0
                sub.house_read = 0

            # Data Visualisation
            # Plot Erasing Functionality
            if sub.solar_time_count == 266 and control.real_time_plotting is False and plot_erase is False:
                plot_erase = True
                solar_erase_index = sub.solar_int
                house_erase_index = sub.house_int
                control_erase_index = control.control_int
            elif (time.time() - sub.initial_time) / 3600 >= 24 and control.real_time_plotting and plot_erase is False:
                plot_erase = True
                solar_erase_index = sub.solar_int
                house_erase_index = sub.house_int
                control_erase_index = control.control_int

            if plot_erase:
                house_plot_index = (sub.house_int - house_erase_index) + 1
                solar_plot_index = (sub.solar_int - solar_erase_index) + 1
                control_plot_index = (control.control_int - control_erase_index) + 1

            # Real Time Line Plotting
            if control.real_time_plotting:
                house_x = sub.data["house_real_time"][house_plot_index:]
                house_y = sub.data["house_plot"][house_plot_index:]
                solar_x = sub.data["solar_real_time"][solar_plot_index:]
                solar_y = sub.data["solar_plot"][solar_plot_index:]
                battery_x = control.control_data["control_real_time"][control_plot_index:]
                battery_y = control.control_data["bat_plot"][control_plot_index:]
                if control.display_grid:
                    grid_x = control.control_data["control_real_time"][control_plot_index:]
                    grid_y = control.control_data["grid_plot"][control_plot_index:]
            else:
                house_x = sub.data["house_time"][house_plot_index:]
                house_y = sub.data["house_plot"][house_plot_index:]
                solar_x = sub.data["solar_time"][solar_plot_index:]
                solar_y = sub.data["solar_plot"][solar_plot_index:]
                battery_x = control.control_data["control_time"][control_plot_index:]
                battery_y = control.control_data["bat_plot"][control_plot_index:]
                if control.display_grid:
                    grid_x = control.control_data["control_time"][control_plot_index:]
                    grid_y = control.control_data["grid_plot"][control_plot_index:]

            house_line.set_data(house_x, house_y)
            solar_line.set_data(solar_x, solar_y)
            battery_line.set_data(battery_x, battery_y)
            if control.display_grid:
                grid_line.set_data(grid_x, grid_y)
            plt.pause(0.001)




