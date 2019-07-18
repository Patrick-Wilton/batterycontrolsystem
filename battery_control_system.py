
import zmq
import time
import threading

import matplotlib.pyplot as plt

from simulation_client import SunSpecDriver


class Subscriber:
    def __init__(self):

        self.battery_read = 0
        self.solar_read = 0
        self.house_read = 0

        self.bat_SOC = 0
        self.solar_power = 0
        self.house_power = 0

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
            self.bat_SOC = int(self.bat_SOC)
            self.battery_read = 1

    def solar_subscriber(self):
        while True:
            solar_string = self.solar_socket.recv()
            s_topic, self.solar_power = solar_string.split()
            self.solar_power = int(self.solar_power)
            self.solar_read = 1

    def house_subscriber(self):
        while True:
            house_string = self.house_socket.recv()
            h_topic, self.house_power = house_string.split()
            self.house_power = int(self.house_power)
            self.house_read = 1


class Control:
    def __init__(self):

        # Control Settings and Initial Values
        self.time_interval = 5  # minutes
        self.time_interval_hour = 1 / (60 / self.time_interval)
        self.time_max_count = int(24 / self.time_interval_hour)
        self.time_count = 1
        self.bat_power = 0
        self.control_interval = 1
        self.control_time = False
        self.solar_filtering = False
        self.house_filtering = False
        self.bat_filtering = True

        # Sets up Kalman Filters
        self.solar_filter = KalmanFilter(1, 0, 1, 0, 1, 0.05, 1)
        self.house_filter = KalmanFilter(1, 0, 1, 700, 1, 0.05, 1)
        self.battery_filter = KalmanFilter(1, 0, 1, 0, 1, 0.05, 1)

        # Creates Internal Data Store
        self.data = dict()
        self.data["soc"] = []
        self.data["bat_power"] = []
        self.data["solar_power"] = []
        self.data["house_power"] = []

        # Sets Initial Plot
        plt.axis([0, 24, -6, 8])
        plt.title('One Day')
        plt.xlabel('Time (Hours)')
        plt.ylabel('Power (kW)')

        # ZeroMQ Publishing
        pub_port = "8093"
        self.pub_topic = 0
        pub_context = zmq.Context()
        self.pub_socket = pub_context.socket(zmq.PUB)
        self.pub_socket.bind("tcp://*:%s" % pub_port)

        # Erases Text File Contents
        open("control_power_values.txt", "w+").close()

    def battery_control(self, bat, solar, house):

        # Data Filtering
        if self.solar_filtering:
            self.solar_filter.step(0, solar)
            solar = self.solar_filter.current_state()
        if self.house_filtering:
            self.house_filter.step(0, house)
            house = self.house_filter.current_state()

        # Internal Data Store
        self.data_store(sub.bat_SOC, sub.solar_power, sub.house_power)

        # Text File Writing
        self.write_to_text(bat, solar, house)

        # Data Plotting
        if self.time_count == self.time_max_count:
            plt.scatter(24, self.bat_power / 1000, s=2, c='g')
            plt.scatter(24, solar / 1000, s=2, c='r')
            plt.scatter(24, house / 1000, s=2, c='b')
            plt.pause(0.05)
            self.time_count = 1
        else:
            plt.scatter(self.time_count / (60 / self.time_interval), self.bat_power / 1000, s=2, c='g')
            plt.scatter(self.time_count / (60 / self.time_interval), solar / 1000, s=2, c='r')
            plt.scatter(self.time_count / (60 / self.time_interval), house / 1000, s=2, c='b')
            plt.pause(0.05)
            self.time_count += 1

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

    def write_to_text(self, bat, solar, house):
        file = open("control_power_values.txt", "a+")
        hour_string = str(round(self.time_count / (60 / self.time_interval), 2))
        soc_str = str(bat)
        solar_str = str(solar)
        house_str = str(house)
        file.write("\n"+hour_string+" "+soc_str+" "+str(self.bat_power)+" "+solar_str+" "+house_str)
        file.close()

    def data_store(self, bat, solar, house):
        self.data["soc"].append(bat)
        self.data["bat_power"].append(self.bat_power)
        self.data["solar_power"].append(solar)
        self.data["house_power"].append(house)


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

    # Starts Subscribers and SunSpec Drivers
    sub = Subscriber()
    sun_spec_driver = SunSpecDriver()

    # MAIN LOOP
    print('Starting Control System')
    while True:
        if control.control_time:

            control.battery_control(sub.bat_SOC, sub.solar_power, sub.house_power)
            time.sleep(control.control_interval)

        elif sub.battery_read == 1 & sub.solar_read == 1 & sub.house_read == 1:

            control.battery_control(sub.bat_SOC, sub.solar_power, sub.house_power)

            # Reset Reads
            sub.battery_read = 0
            sub.solar_read = 0
            sub.house_read = 0




