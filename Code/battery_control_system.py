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
import yaml
import threading
import numpy as np

import matplotlib.pyplot as plt

from Code.simulation_client import SunSpecDriver


class Settings:
    def __init__(self):
        with open("config_settings.yml", 'r') as ymlfile:
            yml_dict = yaml.load(ymlfile, Loader=yaml.FullLoader)
        for k, v in yml_dict.items():
            setattr(self, k, v)


class Subscriber:
    def __init__(self):

        # Settings and Initial Values
        self.battery_read = 0
        self.solar_read = 0
        self.house_read = 0

        self.bat_SOC = 0
        self.solar_power = 0
        self.house_power = 0

        self.day_count = 0

        self.solar_int = 0
        self.house_int = 0

        self.ts = 5000

        # Sets Initial Time
        self.initial_time = None

        self.lock = threading.Lock()

        self.bat_timing = 5  # minutes
        self.solar_timing = 5  # minutes
        self.house_timing = 5  # minutes

        # Erases Previous Text File Contents
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

        # Sets up Kalman Filters
        self.solar_filtering = True
        self.house_filtering = True
        self.solar_filter = KalmanFilter(1, 0, 1, 0, 1, 0.05, 1)
        self.house_filter = KalmanFilter(1, 0, 1, 700, 1, 0.05, 1)

        # ZeroMQ Settings
        self.bat_port = "8090"
        self.solar_port = "8091"
        self.house_port = "8092"
        self.battery_topic = "0"
        self.solar_topic = "0"
        self.house_topic = "0"

        # Defines Sockets and Threads
        self.bat_socket = None
        self.solar_socket = None
        self.house_socket = None

        self.bat_thread = None
        self.solar_thread = None
        self.house_thread = None

        # Starts Subscribers
        self.start_subscribers()

    def start_subscribers(self):
        sub_context = zmq.Context()
        self.bat_socket = sub_context.socket(zmq.SUB)
        self.bat_socket.connect("tcp://localhost:%s" % self.bat_port)
        self.bat_socket.setsockopt_string(zmq.SUBSCRIBE, self.battery_topic)
        self.solar_socket = sub_context.socket(zmq.SUB)
        self.solar_socket.connect("tcp://localhost:%s" % self.solar_port)
        self.solar_socket.setsockopt_string(zmq.SUBSCRIBE, self.solar_topic)
        self.house_socket = sub_context.socket(zmq.SUB)
        self.house_socket.connect("tcp://localhost:%s" % self.house_port)
        self.house_socket.setsockopt_string(zmq.SUBSCRIBE, self.house_topic)

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

                self.write_to_text("SOC", round((time.time() - self.initial_time) * self.ts, 2) / 3600, self.bat_SOC)
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

                self.write_to_text("solar", round((time.time() - self.initial_time) * self.ts, 2) / 3600, self.solar_power)

                # self.data["solar_time"].append(self.solar_time_count / (60 / self.solar_timing))

                self.data["solar_time"].append((round((time.time() - self.initial_time) * self.ts, 2) / 3600) - (24 * self.day_count))
                self.data["solar_power"].append(self.solar_power / 1000)

                if abs(self.data["solar_time"][-1] - 24) < 0.1:
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

                self.write_to_text("house", round((time.time() - self.initial_time) * self.ts, 2) / 3600, self.house_power)

                # self.data["house_time"].append(self.house_time_count / (60 / self.house_timing))
                self.data["house_time"].append((round((time.time() - self.initial_time) * self.ts, 2) / 3600) - (24 * self.day_count))

                self.data["house_power"].append(self.house_power / 1000)

                if abs(self.data["house_time"][-1] - 24) < 0.1:
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

        # Initial Values
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

        self.ts = 5000

        # Initial Values
        self.control_interval = 5  # minutes
        self.sub_time = sub_time
        time_interval_hour = 1 / (60 / self.control_interval)
        self.time_max_count = int(24 / time_interval_hour)
        self.control_time_count = 1
        self.initial_time = None
        self.control_int = 0
        self.day_count = 0

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
        # self.control_data["control_time"].append((curr_time - 1) / (60 / self.sub_time))

        self.control_data["control_time"].append(
            (round((time.time() - self.initial_time) * self.ts, 2) / 3600) - 24 * self.day_count)
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


class DataVisualisation:
    def __init__(self, ref):

        # Plot Settings
        self.display_grid = True

        # Sets Initial Plot Parameters
        plt.figure(figsize=[12, 7])
        plt.axis([0, 24, -6, 8])
        plt.title('One Day')
        plt.xlabel('Time (Hours)')
        plt.ylabel('Power (kW)')
        plt.grid(True)
        plt.ion()

        # Creates Reference Line
        ref_line = plt.hlines(ref / 1000, 0, 24, linestyles='dashed')
        ref_line.set_label('Reference Grid Power')

        # Initialises Line Graphs
        self.house_line, = plt.plot([], [], '-o', alpha=0.8, c='b', markersize=2)
        self.house_line.set_label('House Power')
        self.solar_line, = plt.plot([], [], '-o', alpha=0.8, c='r', markersize=2)
        self.solar_line.set_label('Solar Power')
        self.battery_line, = plt.plot([], [], '-o', alpha=0.8, c='g', markersize=2)
        self.battery_line.set_label('Battery Power')
        self.grid_line, = plt.plot([], [], '-o', alpha=0.8, c='m', markersize=2)
        if self.display_grid:
            self.grid_line.set_label('Grid Power')
        plt.legend()

    def update_plot(self, sub_data, control_data, index):

        # Sets x and y values for house, solar and battery lines
        house_x = sub_data["house_time"][index[0]:]
        house_y = sub_data["house_plot"][index[0]:]
        solar_x = sub_data["solar_time"][index[1]:]
        solar_y = sub_data["solar_plot"][index[1]:]
        battery_x = control_data["control_time"][index[2]:]
        battery_y = control_data["bat_plot"][index[2]:]

        # Updates house, solar and battery lines
        self.house_line.set_data(house_x, house_y)
        self.solar_line.set_data(solar_x, solar_y)
        self.battery_line.set_data(battery_x, battery_y)

        # Updates grid line if necessary
        if self.display_grid:
            grid_x = control_data["control_time"][index[2]:]
            grid_y = control_data["grid_plot"][index[2]:]
            self.grid_line.set_data(grid_x, grid_y)

        # Update plot
        plt.pause(0.001)


if __name__ == '__main__':

    # Reads settings configuration file
    settings = Settings()

    # Creates Initial Connect Parameters
    plot_erase = False
    control_check = False

    initial_connect = False
    connected = False

    plot_index = [0, 0, 0]

    house_erase_index = 0
    solar_erase_index = 0
    control_erase_index = 0

    ts = 5000

    # Starts Subscribers, SunSpec Drivers and Control Class
    data = Subscriber()
    sun_spec_driver = SunSpecDriver(settings)
    control = Control(data.solar_timing)
    plot = DataVisualisation(control.grid_ref / 1000)

    print('Starting Control System')

    # FIRST CONNECTION LOOP
    while connected is False:

        # Checks for Initial Connection Values
        bat_connect = data.bat_SOC == b'bat_connect'
        solar_connect = data.solar_power == b'solar_connect'
        house_connect = data.house_power == b'house_connect'

        connecting = bat_connect and solar_connect and house_connect
        residual_connect = bat_connect or solar_connect or house_connect

        # Runs if Initial Connection is Established
        if connecting:
            control.pub_socket.send_string("%d %s" % (control.pub_topic, 'connected'))
            test = time.time()
            data.initial_time = round(time.time(), 2)
            control.initial_time = data.initial_time
            initial_connect = True

        # Runs if all Subscribers start returning intended values
        if initial_connect and residual_connect is False:
            print((24 / 288) / ((time.time() - test) / 3600))
            connected = True

    # MAIN LOOP
    while True:

        # Increases day counters
        print(((time.time() - data.initial_time) * ts) / 3600)
        if (((time.time() - data.initial_time) * ts) / 3600) - (24 * data.day_count) >= 24:
            control.day_count += 1
            data.day_count += 1

        # CONTROL LOOP
        all_read = data.battery_read == 1 and data.solar_read == 1 and data.house_read == 1
        if control.control_time:

            # True every control time step
            control_step = int((time.time() - data.initial_time) * ts) % (control.control_time_step * 60) == 0

            # Ensures control isn't applied twice on same condition met
            if control_step and control_check is False:
                control.battery_control(data.bat_SOC, data.solar_power, data.house_power, data.solar_time_count)
                control_check = True
            else:
                control_check = False

        # Used mainly for simulations
        elif all_read:

            # Applies Control
            control.battery_control(data.bat_SOC, data.solar_power, data.house_power, data.solar_time_count)

            # Resets Subscriber Read Values
            data.battery_read = 0
            data.solar_read = 0
            data.house_read = 0

        # DATA VISUALISATION
        # Plot Erasing Functionality
        if ((time.time() - data.initial_time) * ts) / 3600 >= 22:
            if plot_erase is False:
                solar_erase_index = data.solar_int
                house_erase_index = data.house_int
                control_erase_index = control.control_int
                plot_erase = True
            plot_index[0] = (data.house_int - house_erase_index) + 1
            plot_index[1] = (data.solar_int - solar_erase_index) + 1
            plot_index[2] = (control.control_int - control_erase_index) + 1

        # Real Time Line Plotting
        plot.update_plot(data.data, control.control_data, plot_index)





