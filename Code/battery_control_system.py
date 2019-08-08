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
    def __init__(self, config_settings):

        # Obtains settings from cofig file
        self.settings = config_settings

        # Parameters and Initial Values
        self.battery_read = 0
        self.solar_read = 0
        self.house_read = 0

        self.bat_SOC = 0
        self.solar_power = 0
        self.house_power = 0

        self.day_count = 0
        self.soc_num = 0
        self.solar_num = 0
        self.house_num = 0
        self.initial_time = 0

        # Creates Thread Lock
        self.lock = threading.Lock()

        # Erases Previous Text File Contents
        open("control_power_values.txt", "w+").close()

        # Creates Internal Data Store
        self.data_store = dict()
        self.data_store["soc_time"] = list()
        self.data_store["soc_value"] = list()
        self.data_store["soc_plot"] = list()
        self.data_store["solar_time"] = list()
        self.data_store["solar_value"] = list()
        self.data_store["solar_plot"] = list()
        self.data_store["house_time"] = list()
        self.data_store["house_value"] = list()
        self.data_store["house_plot"] = list()

        # Sets up Kalman Filters
        self.solar_filter = KalmanFilter(1, 0, 1, 0, 1, 0.05, 1)
        self.house_filter = KalmanFilter(1, 0, 1, 700, 1, 0.05, 1)

        # Defines Sockets and Threads
        self.bat_socket = None
        self.solar_socket = None
        self.house_socket = None

        self.bat_thread = None
        self.solar_thread = None
        self.house_thread = None

        # Starts Subscribers
        self.start_subscribers()

    def write_to_text(self, device, curr_time, value):

        self.lock.acquire()

        file = open("control_power_values.txt", "a+")

        file.write("\n"+device+" "+str(curr_time)+" "+str(value))
        file.close()

        self.lock.release()

    def update_data_store(self, device, value):

        # New Value
        self.data_store[device + "_value"].append(value / 1000)

        # Time Value
        if self.settings.simulation["use_real_time"]:
            curr_time = (round(time.time() - self.initial_time, 2) / 3600) - (24 * self.day_count)
            self.data_store[device + "_time"].append(curr_time)
        else:
            curr_time = round((self.data_store[device + "_time"][-1] +
                               self.settings.simulation["time_step"] / 60) - (24 * self.day_count))
            self.data_store[device + "_time"].append(curr_time)

        # Plot Value
        if 24 - self.data_store[device + "_time"][-1] >= 0.1:
            self.data_store[device + "_plot"].append(np.nan)
        else:
            self.data_store[device + "_plot"].append(value / 1000)

    def start_subscribers(self):

        # Connects Sockets
        sub_context = zmq.Context()
        self.bat_socket = sub_context.socket(zmq.SUB)
        self.bat_socket.connect("tcp://localhost:%s" % self.settings.ZeroMQ["battery_SOC_port"])
        self.bat_socket.setsockopt_string(zmq.SUBSCRIBE, str(self.settings.ZeroMQ["battery_SOC_topic"]))
        self.solar_socket = sub_context.socket(zmq.SUB)
        self.solar_socket.connect("tcp://localhost:%s" % self.settings.ZeroMQ["solar_port"])
        self.solar_socket.setsockopt_string(zmq.SUBSCRIBE, str(self.settings.ZeroMQ["solar_topic"]))
        self.house_socket = sub_context.socket(zmq.SUB)
        self.house_socket.connect("tcp://localhost:%s" % self.settings.ZeroMQ["house_port"])
        self.house_socket.setsockopt_string(zmq.SUBSCRIBE, str(self.settings.ZeroMQ["house_topic"]))

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
            # Obtains Value from Topic
            bat_string = self.bat_socket.recv()
            b_topic, self.bat_SOC = bat_string.split()

            # Runs if not connecting
            if self.bat_SOC != b'bat_connect':

                # Converts to int
                self.bat_SOC = int(self.bat_SOC)

                # Updates Text File and Data Store
                curr_time = (round(time.time() - self.initial_time, 2) / 3600) - (24 * self.day_count)
                self.write_to_text("SOC", curr_time, self.bat_SOC)
                self.update_data_store("soc", self.bat_SOC)

                # Sets Reading and Increases Counter
                self.soc_num += 1
                self.battery_read = 1

    def solar_subscriber(self):
        while True:
            # Obtains Value from Topic
            solar_string = self.solar_socket.recv()
            s_topic, self.solar_power = solar_string.split()

            # Runs if not connecting
            if self.solar_power != b'solar_connect':

                # Converts to int
                self.solar_power = int(self.solar_power)

                # Applies Filtering
                if self.settings.control["solar_filtering"]:
                    self.solar_filter.step(0, self.solar_power)
                    self.solar_power = self.solar_filter.current_state()

                # Updates Text File and Data Store
                curr_time = (round(time.time() - self.initial_time, 2) / 3600) - (24 * self.day_count)
                self.write_to_text("solar", curr_time, self.solar_power)
                self.update_data_store("solar", self.solar_power)

                # Sets Reading and Increases Counter
                self.solar_num += 1
                self.solar_read = 1

    def house_subscriber(self):
        while True:
            # Obtains Value from Topic
            house_string = self.house_socket.recv()
            h_topic, self.house_power = house_string.split()

            # Runs if not connecting
            if self.house_power != b'house_connect':

                # Converts to int
                self.house_power = int(self.house_power)

                # Applies Filtering
                if self.settings.control["house_filtering"]:
                    self.house_filter.step(0, self.house_power)
                    self.house_power = self.house_filter.current_state()

                # Updates Text File and Data Store
                curr_time = (round(time.time() - self.initial_time, 2) / 3600) - (24 * self.day_count)
                self.write_to_text("house", curr_time, self.house_power)
                self.update_data_store("house", self.house_power)

                print(1)

                # Sets Reading and Increases Counter
                self.house_num += 1
                self.house_read = 1


class Publisher:
    def __init__(self, config_settings):

        # Obtains config file settings and starts Subscribers
        self.settings = config_settings

        # Initial Parameters
        self.pub_num = 0
        self.bat_power = 0
        self.grid = 0
        self.initial_time = 0
        self.day_count = 0

        # Non-Optimiser Control Settings
        self.grid_ref = self.settings.simulation["grid_ref"]
        self.grid_control_dir = self.settings.simulation["control_dir"]

        # Sets up Kalman Filters
        self.battery_filter = KalmanFilter(1, 0, 1, 0, 1, 0.05, 1)

        # Creates Internal Data Store
        self.data_store = dict()
        self.data_store["bat_power"] = list()
        self.data_store["bat_plot"] = list()
        self.data_store["bat_time"] = list()
        self.data_store["grid_power"] = list()
        self.data_store["grid_plot"] = list()
        self.data_store["grid_time"] = list()

        # ZeroMQ Publishing
        pub_context = zmq.Context()
        self.pub_socket = pub_context.socket(zmq.PUB)
        self.pub_socket.bind("tcp://*:%s" % str(self.settings.ZeroMQ["battery_power_port"]))

    def set_power(self, bat_power):
        self.bat_power = bat_power

    def set_grid(self, solar, house):
        self.grid = self.bat_power + solar + house

    def update_data_store(self, device, power):

        # Power Value
        self.data_store[device + "_power"].append(power / 1000)

        # Time Value
        if self.settings.simulation["use_real_time"]:
            curr_time = (round(time.time() - self.initial_time, 2) / 3600) - (24 * self.day_count)
            self.data_store[device + "_time"].append(curr_time)
        else:
            curr_time = round((self.data_store[device + "_time"][-1] +
                               self.settings.simulation["time_step"] / 60) - (24 * self.day_count))
            self.data_store[device + "_time"].append(curr_time)

        # Plot Value
        if 24 - self.data_store[device + "_time"][-1] >= 0.1:
            self.data_store[device + "_plot"].append(np.nan)
        else:
            self.data_store[device + "_plot"].append(power / 1000)

    def non_optimiser_control(self, soc):

        # Above, Below and Both Control
        above_control = self.grid_control_dir == "Above" and self.grid > self.grid_ref
        below_control = self.grid_control_dir == "Below" and self.grid < self.grid_ref

        # Sets new Battery Power
        if above_control or below_control or self.grid_control_dir == "Both":
            self.set_power(-self.grid + self.bat_power + self.grid_ref)

        # Accounting for SOC
        if (soc == 0 and self.bat_power < 0) or (soc == 100 and self.bat_power > 0):
            self.set_power(0)

        # Battery Power Filtering
        if self.settings.control["battery_filtering"]:
            self.battery_filter.step(0, self.bat_power)
            self.set_power(self.battery_filter.current_state())

    def publish_power(self):

        # Publishing
        self.pub_socket.send_string("%d %d" % (self.settings.ZeroMQ["battery_power_topic"], self.bat_power))

        # Increases Publish Counter
        self.pub_num += 1


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


class DataVisualisation:
    def __init__(self, config_settimgs):

        # Obtains Settings from Config File
        self.settings = config_settimgs

        # Plot Settings and Initial Values
        self.display_grid = self.settings.simulation["display_grid"]
        self.initial_time = 0
        self.day_count = 0
        self.plot_erase = False

        self.b_index = 0
        self.s_index = 0
        self.h_index = 0
        self.g_index = 0
        self.p_index = 0
        self.plot_index = [0, 0, 0, 0, 0]  # bat, solar, house, grid, power

        # Sets Initial Plot Parameters
        plt.figure(figsize=[12, 7])
        plt.axis([0, 24, -6, 8])
        plt.title('One Day')
        plt.xlabel('Time (Hours)')
        plt.ylabel('Power (kW)')
        plt.grid(True)
        plt.ion()

        # Creates Reference Line
        ref_line = plt.hlines(self.settings.simulation["grid_ref"] / 1000, 0, 24, linestyles='dashed')
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

    def update_erase_index(self, house_time, b, s, h, g, p):

        # Obtains the current time
        if self.settings.simulation["use_real_time"]:
            curr_time = (round(time.time() - self.initial_time, 2) / 3600) - (24 * self.day_count)
        else:
            curr_time = house_time

        if curr_time >= 22:
            if self.plot_erase is False:
                self.b_index = b
                self.s_index = s
                self.h_index = h
                self.g_index = g
                self.p_index = p
                self.plot_erase = True
            self.plot_index[0] = (b - self.b_index) + 1
            self.plot_index[1] = (s - self.s_index) + 1
            self.plot_index[2] = (h - self.h_index) + 1
            self.plot_index[3] = (g - self.g_index) + 1
            self.plot_index[4] = (p - self.p_index) + 1

    def update_plot(self, sub_data, pub_data):

        # Sets x and y values for house, solar and battery lines
        solar_x = sub_data["solar_time"][self.plot_index[1]:]
        solar_y = sub_data["solar_plot"][self.plot_index[1]:]
        house_x = sub_data["house_time"][self.plot_index[2]:]
        house_y = sub_data["house_plot"][self.plot_index[2]:]
        battery_x = pub_data["bat_time"][self.plot_index[4]:]
        battery_y = pub_data["bat_plot"][self.plot_index[4]:]

        # Updates house, solar and battery lines
        self.house_line.set_data(house_x, house_y)
        self.solar_line.set_data(solar_x, solar_y)
        self.battery_line.set_data(battery_x, battery_y)

        # Updates grid line if necessary
        if self.display_grid:
            grid_x = pub_data["grid_time"][self.plot_index[3]:]
            grid_y = pub_data["grid_plot"][self.plot_index[3]:]
            self.grid_line.set_data(grid_x, grid_y)

        # Update plot
        plt.pause(0.001)


class ControlLoop:
    def __init__(self, config_settings):

        # Initialises Classes
        self.settings = config_settings
        self.sub = Subscriber(config_settings)
        self.pub = Publisher(config_settings)
        self.plot = DataVisualisation(config_settings)

        # Sets Boolean Parameters
        self.plot_erase = False
        self.control_check = False
        self.initial_connect = False
        self.connected = False

    def update_day_counter(self):

        # Obtains the current time
        if self.settings.simulation["use_real_time"]:
            curr_time = (round(time.time() - self.sub.initial_time, 2) / 3600) - (24 * self.sub.day_count)
        else:
            curr_time = self.sub.data_store["house_time"][-1]

        # Checks if it has been 24 hours
        if curr_time >= 24:
            self.sub.day_count += 1
            self.pub.day_count += 1
            self.plot.day_count += 1

    def no_optimiser_control(self):

        # Calculates new grid value
        self.pub.set_grid(self.sub.solar_power, self.sub.house_power)

        # Updates Data Stores
        self.pub.update_data_store("grid", self.pub.grid)
        self.pub.update_data_store("bat", self.pub.bat_power)

        # Applies Control
        self.pub.non_optimiser_control(self.sub.bat_SOC)

        # Publishes Value
        self.pub.publish_power()

    def connection_loop(self):
        while self.connected is False:

            # Checks for Initial Connection Values
            bat_connect = self.sub.bat_SOC == b'bat_connect'
            solar_connect = self.sub.solar_power == b'solar_connect'
            house_connect = self.sub.house_power == b'house_connect'

            connecting = bat_connect and solar_connect and house_connect
            residual_connect = bat_connect or solar_connect or house_connect

            # Runs if Initial Connection is Established
            if connecting:
                self.pub.pub_socket.send_string("%d %s" % (self.settings.ZeroMQ["battery_power_topic"], 'connected'))
                self.initial_connect = True

            # Runs if all Subscribers start returning intended values
            if self.initial_connect and residual_connect is False:
                self.connected = True
                self.sub.initial_time = round(time.time(), 2)
                self.pub.initial_time = self.sub.initial_time
                self.plot.initial_time = self.sub.initial_time

    def control_loop(self):

        # Checks if all subscribers have new values
        all_read = self.sub.battery_read == 1 and self.sub.solar_read == 1 and self.sub.house_read == 1

        # Runs if event based publishing is OFF
        if self.settings.ZeroMQ["use_event_pub"] is False:
            print('testing')

            # True every control time step
            #control_step = int((time.time() - data.initial_time) * ts) % (control.control_time_step * 60) == 0

            # Ensures control isn't applied twice on same condition met
            #if control_step and control_check is False:
            #    control.battery_control(data.bat_SOC, data.solar_power, data.house_power, data.solar_time_count)
            #    control_check = True
            #else:
            #    control_check = False

        # Used mainly for simulations
        elif all_read:

            # Applies Control
            self.no_optimiser_control()

            # Resets Subscriber Read Values
            self.sub.battery_read = 0
            self.sub.solar_read = 0
            self.sub.house_read = 0


if __name__ == '__main__':

    # Reads settings configuration file
    settings = Settings()
    control = ControlLoop(settings)
    drivers = SunSpecDriver(settings)

    time.sleep(2)

    print('Connecting')
    control.connection_loop()

    print('Starting Control System')
    # MAIN LOOP
    while True:
        control.update_day_counter()
        control.control_loop()
        print(control.sub.data_store["house_time"])
        # control.plot.update_erase_index(control.sub.data_store["house_time"][-1], control.sub.soc_num, control.sub.solar_num, control.sub.house_num, control.pub.pub_num, control.pub.pub_num)
        control.plot.update_plot(control.sub.data_store, control.pub.data_store)









