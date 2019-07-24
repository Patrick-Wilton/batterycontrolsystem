
import zmq
import time
import threading

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

        self.lock = threading.Lock()

        self.bat_timing = 5  # minutes
        self.solar_timing = 5  # minutes
        self.house_timing = 5  # minutes

        # Erases Text File Contents
        open("control_power_values.txt", "w+").close()

        # Creates Internal Data Store
        self.data = dict()
        self.data["soc"] = []
        self.data["solar_power"] = []
        self.data["house_power"] = []

        # Sets up Kalman Filters
        self.solar_filtering = False
        self.house_filtering = False
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
                self.data_store("soc", self.house_power)

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
                self.data_store("solar_power", self.house_power)

                if self.solar_time_count == self.solar_max_time:
                    self.solar_time_count = 1
                else:
                    self.solar_time_count += 1

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
                self.data_store("house_power", self.house_power)

                if self.house_time_count == self.house_max_time:
                    self.house_time_count = 1
                else:
                    self.house_time_count += 1

                self.house_read = 1

    def write_to_text(self, device, curr_time, value):
        self.lock.acquire()

        file = open("control_power_values.txt", "a+")

        file.write("\n"+device+" "+str(curr_time)+" "+str(value))
        file.close()

        self.lock.release()

    def data_store(self, device, value):
        self.data[device].append(value)


class Control:
    def __init__(self):

        # Control Settings and Initial Values
        self.control_interval = 5  # minutes
        time_interval_hour = 1 / (60 / self.control_interval)
        self.time_max_count = int(24 / time_interval_hour)
        self.control_time_count = 1

        self.bat_power = 0
        self.control_time_step = 30
        self.control_time = False
        self.bat_filtering = True

        # Sets up Kalman Filters
        self.battery_filter = KalmanFilter(1, 0, 1, 0, 1, 0.05, 1)

        # Creates Internal Data Store
        self.bat_power_data = list()

        # ZeroMQ Publishing
        pub_port = "8093"
        self.pub_topic = 0
        pub_context = zmq.Context()
        self.pub_socket = pub_context.socket(zmq.PUB)
        self.pub_socket.bind("tcp://*:%s" % pub_port)

    def battery_control(self, bat, solar, house):

        # Internal Data Store
        self.bat_power_data.append(self.bat_power)

        if self.control_time_count == self.time_max_count:
            self.control_time_count = 1
        else:
            self.control_time_count += 1

        # Control System
        grid = self.bat_power + solar + house
        self.bat_power = -grid + self.bat_power
        if (bat == 0 and self.bat_power < 0) or (bat == 100 and self.bat_power > 0):
            self.bat_power = 0
        if self.bat_filtering:
            self.battery_filter.step(0, self.bat_power)
            self.bat_power = self.battery_filter.current_state()

        # Publishing
        self.pub_socket.send_string("%d %d" % (self.pub_topic, self.bat_power))


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


if __name__ == '__main__':

    # Creates Control Class Instance
    control = Control()
    connected = False
    first_read = False

    # Starts Subscribers and SunSpec Drivers
    sub = Subscriber()
    sun_spec_driver = SunSpecDriver()

    # Sets Initial Plot
    plt.axis([0, 24, -6, 8])
    plt.title('One Day')
    plt.xlabel('Time (Hours)')
    plt.ylabel('Power (kW)')

    # MAIN LOOP
    print('Starting Control System')
    while True:
        bat_connect = sub.bat_SOC == b'bat_connect'
        solar_connect = sub.solar_power == b'solar_connect'
        house_connect = sub.house_power == b'house_connect'
        initial_connect = bat_connect and solar_connect and house_connect
        connect_check = bat_connect or solar_connect or house_connect

        all_read = sub.battery_read == 1 and sub.solar_read == 1 and sub.house_read == 1
        if all_read:
            first_read = True

        if initial_connect:
            control.pub_socket.send_string("%d %s" % (control.pub_topic, 'connected'))
            sub.initial_time = time.time()
            connected = True

        elif connected and connect_check is False:
            if control.control_time and first_read:

                control.battery_control(sub.bat_SOC, sub.solar_power, sub.house_power)

            elif all_read:

                control.battery_control(sub.bat_SOC, sub.solar_power, sub.house_power)

                # Reset Reads
                sub.battery_read = 0
                sub.solar_read = 0
                sub.house_read = 0

            # Plotting
            plt.scatter(control.control_time_count / (60 / control.control_interval), control.bat_power / 1000, s=2, c='g')
            plt.scatter(sub.solar_time_count / (60 / sub.solar_timing), int(sub.solar_power) / 1000, s=2, c='r')
            plt.scatter(sub.house_time_count / (60 / sub.house_timing), int(sub.house_power) / 1000, s=2, c='b')
            plt.pause(0.01)




