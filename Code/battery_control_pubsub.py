
import threading
import time

import numpy as np
import zmq

from Code.kalman_filter import KalmanFilter


class Subscriber:
    def __init__(self, config_settings):

        # Obtains settings from config file
        self.settings = config_settings

        # Parameters and Initial Values
        self.battery_read = 0
        self.solar_read = 0
        self.house_read = 0

        self.bat_SOC = self.settings.battery["initial_SOC"]
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
        if device == "soc":
            self.data_store[device + "_value"].append(value)
        else:
            self.data_store[device + "_value"].append(value / 1000)

        # Time Value
        if self.settings.simulation["use_real_time"]:
            curr_time = (round(time.time() - self.initial_time, 2) / 3600) - (24 * self.day_count)
        elif bool(self.data_store[device + "_time"]) is False:
            curr_time = 0
        else:
            curr_time = self.data_store[device + "_time"][-1] + self.settings.simulation["time_step"] / 60
            if abs(curr_time - 24) < 0.02:
                curr_time = 0
        self.data_store[device + "_time"].append(curr_time)

        # Plot Value
        if 24 - self.data_store[device + "_time"][-1] <= 0.1:
            self.data_store[device + "_plot"].append(np.nan)
        else:
            if device == "soc":
                self.data_store[device + "_plot"].append(value / (100 / 6))
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

                # Sets Read and Increases Counter
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

                # Sets Read and Increases Counter
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

                # Sets Read and Increases Counter
                self.house_num += 1
                self.house_read = 1


class Publisher:
    def __init__(self, config_settings):

        # Obtains config file settings and starts Subscribers
        self.settings = config_settings

        # Initial Parameters
        self.bat_num = 0
        self.grid_num = 0
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

    def update_data_store(self, device, power, house_time):

        # Power Value
        self.data_store[device + "_power"].append(power / 1000)

        # Time Value
        if self.settings.simulation["use_real_time"]:
            curr_time = (round(time.time() - self.initial_time, 2) / 3600) - (24 * self.day_count)
        else:
            curr_time = house_time
        self.data_store[device + "_time"].append(curr_time)

        # Plot Value
        if 24 - self.data_store[device + "_time"][-1] < 0.1:
            self.data_store[device + "_plot"].append(np.nan)
        else:
            self.data_store[device + "_plot"].append(power / 1000)

        # Increase total counters
        if device == "bat":
            self.bat_num += 1
        else:
            self.grid_num += 1

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
        self.pub_socket.send_string("%d %d" % (self.settings.ZeroMQ["battery_power_topic"], self.bat_power))