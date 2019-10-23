

import time
import yaml
import numpy as np

import matplotlib.pyplot as plt

from Code.system_drivers import SunSpecDriver
from Code.optimiser_model import Optimiser
from Code.kalman_filter import KalmanFilter
from Code.battery_control_pubsub import Publisher, Subscriber


class Settings:
    def __init__(self):
        with open("config_settings.yml", 'r') as yml_file:
            yml_dict = yaml.load(yml_file, Loader=yaml.FullLoader)
        for k, v in yml_dict.items():
            setattr(self, k, v)


class DataVisualisation:
    def __init__(self, config_settings):

        # Obtains Settings from Config File
        self.settings = config_settings

        # Plot Settings and Initial Values
        self.display_grid = self.settings.simulation["display_grid"]
        self.display_soc = self.settings.simulation["display_SOC"]
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

        # Creates Reference Lines
        if self.settings.control["optimiser"]:
            soc_ref = plt.hlines(6, 0, 24, linestyles='dashed')
            soc_ref.set_label('100% State of Charge')
        else:
            ref_line = plt.hlines(self.settings.simulation["grid_ref"] / 1000, 0, 24, linestyles='dashed')
            ref_line.set_label('Reference Grid Power')
            soc_ref = plt.hlines(6, 0, 24, linestyles='dashed')
            soc_ref.set_label('100% State of Charge')

        # Initialises Line Graphs
        self.soc_line, = plt.plot([], [], '-o', alpha=0.8, c='y', markersize=2)
        if self.display_soc:
            self.soc_line.set_label('State of Charge')
        self.grid_line, = plt.plot([], [], '-o', alpha=0.8, c='m', markersize=2)
        if self.display_grid:
            self.grid_line.set_label('Grid Power')
        self.battery_line, = plt.plot([], [], '-o', alpha=0.8, c='g', markersize=2)
        self.battery_line.set_label('Battery Power')
        self.house_line, = plt.plot([], [], '-o', alpha=0.8, c='b', markersize=2)
        self.house_line.set_label('House Power')
        self.solar_line, = plt.plot([], [], '-o', alpha=0.8, c='r', markersize=2)
        self.solar_line.set_label('Solar Power')
        plt.legend()

    def update_erase_index(self, house_time, b, s, h, g, p):

        # Obtains the current time
        if self.settings.simulation["use_real_time"]:
            curr_time = (round(time.time() - self.initial_time, 2) / 3600) - (24 * self.day_count)
        elif bool(house_time) is False:
            curr_time = 0
        else:
            curr_time = house_time[-1]

        if curr_time >= 22 and self.plot_erase is False:
            self.b_index = b
            self.s_index = s
            self.h_index = h
            self.g_index = g
            self.p_index = p
            self.plot_erase = True

        if self.plot_erase:
            self.plot_index[0] = (b - self.b_index) + 1
            self.plot_index[1] = (s - self.s_index) + 1
            self.plot_index[2] = (h - self.h_index) + 1
            self.plot_index[3] = (g - self.g_index) + 1
            self.plot_index[4] = (p - self.p_index) + 1

    def update_plot(self, sub_data, pub_data):

        # Update soc line if necessary
        if self.display_soc:
            soc_x = sub_data["soc_time"][self.plot_index[0]:]
            soc_y = sub_data["soc_plot"][self.plot_index[0]:]
            self.soc_line.set_data(soc_x, soc_y)

        # Updates grid line if necessary
        if self.display_grid:
            grid_x = pub_data["grid_time"][self.plot_index[3]:]
            grid_y = pub_data["grid_plot"][self.plot_index[3]:]
            self.grid_line.set_data(grid_x, grid_y)

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

        # Update plot
        plt.pause(0.001)


class ControlSystem:
    def __init__(self, config_settings):

        # Initialises Classes
        self.settings = config_settings
        self.sub = Subscriber(config_settings)
        self.pub = Publisher(config_settings)
        self.optimiser = Optimiser(config_settings)
        if self.settings.simulation["use_visualisation"]:
            self.plot = DataVisualisation(config_settings)

        # Sets Boolean Parameters
        self.control_mod = False
        self.opt_mod = False
        self.data_mod = False
        self.initial_connect = False
        self.connected = False

        # Total Energy
        self.solar_energy = 0
        self.house_energy = 0

        # Sets internal parameters
        self.mod_thresh = 0.002
        self.optimiser_index = 0
        self.data_skip = 0
        self.covariance = 0.3
        self.power_cov = 0.5
        self.power_scale = self.settings.control["power_covariance"]
        self.prev_house_data = None
        self.prev_house_control = None
        self.prev_house_opt = None
        self.prev_house_day = None
        self.time_step = self.settings.control["data_time_step"]
        self.control_step = self.settings.control["control_time_step"]
        self.opt_step = self.settings.control["optimiser_time_step"]

        # Creates 24 hour data stores and filters
        self.power = None
        self.load = list(self.optimiser.load)
        self.pv = list(self.optimiser.pv)
        self.import_tariff = list(self.optimiser.import_tariff.values())
        self.export_tariff = list(self.optimiser.export_tariff.values())

        self.load_filter = None
        self.pv_filter = None
        self.power_filter = KalmanFilter(1, 0, 1, 0, 1, self.power_cov, 1)

    def current_time(self):

        # Obtains the current time
        if self.settings.simulation["use_real_time"]:
            curr_time = (round(time.time() - self.sub.initial_time, 2) / 3600) - (24 * self.sub.day_count)
        elif bool(self.sub.data_store["house_time"]) is False:
            curr_time = 0
        else:
            curr_time = self.sub.data_store["house_time"][-1]
        return curr_time

    def update_day_counter(self):

        # Obtains the current time
        curr_time = self.current_time()

        # Obtains current house time
        if bool(self.sub.data_store["house_time"]) is False:
            curr_house_time = 0
        else:
            curr_house_time = self.sub.data_store["house_time"][-1]

        if 1 < curr_time < 23:
            self.prev_house_day = None

        # Checks if it has been 24 hours
        if abs(curr_time - 24) < 0.1 and curr_house_time != self.prev_house_day:
            self.sub.day_count += 1
            self.pub.day_count += 1
            if self.settings.simulation["use_visualisation"]:
                self.plot.day_count += 1
            self.prev_house_day = curr_house_time

    def calculate_savings(self):

        # Energy Totals
        self.solar_energy += (self.sub.solar_power / 1000) * (self.time_step / 60)
        self.house_energy += (self.sub.house_power / 1000) * (self.time_step / 60)

        # Total Savings
        if self.pub.grid < 0:
            self.pub.savings += (self.pub.grid / 1000) * (self.time_step / 60) * self.export_tariff[0]
        else:
            self.pub.savings += (self.pub.grid / 1000) * (self.time_step / 60) * self.import_tariff[0]

        # Battery Exclusive Savings
        if self.pub.sol_grid < 0:
            self.pub.sol_savings += (self.pub.sol_grid / 1000) * (self.time_step / 60) * self.export_tariff[0]
        else:
            self.pub.sol_savings += (self.pub.sol_grid / 1000) * (self.time_step / 60) * self.import_tariff[0]

        # No PV System Cost
        self.pub.house_import += (self.sub.house_power / 1000) * (self.time_step / 60) * self.import_tariff[0]

    def update_24_data(self, data_skip):

        # Applies filters and removes last entries (solar and load)
        if len(self.load) == (60 / self.time_step) * 24:

            for i in reversed(range(0, data_skip + 1)):
                # Create Filters
                self.load_filter = KalmanFilter(1, 0, 1, self.load[0], 1, self.covariance, 1)
                self.pv_filter = KalmanFilter(1, 0, 1, self.pv[0], 1, self.covariance, 1)

                # Apply Filters
                self.load_filter.step(0, self.sub.data_store["house_value"][-(i + 1)] / (60 / self.time_step))
                self.pv_filter.step(0, self.sub.data_store["solar_value"][-(i + 1)] / (60 / self.time_step))

                # Remove First Value
                self.load.pop(0)
                self.pv.pop(0)

                # Append new values
                self.load.append(self.load_filter.current_state())
                self.pv.append(self.pv_filter.current_state())
        else:
            # Append new values
            self.load.append(self.sub.house_power / (1000 * (60 / self.time_step)))
            self.pv.append(self.sub.solar_power / (1000 * (60 / self.time_step)))

        # Update Tariffs
        for i in range(0, data_skip + 1):
            self.import_tariff.append(self.import_tariff.pop(0))
            self.export_tariff.append(self.export_tariff.pop(0))

        # Update profile classes and energy system
        self.optimiser.update_profiles(np.array(self.load),
                                       np.array(self.pv),
                                       np.array(self.import_tariff),
                                       np.array(self.export_tariff),
                                       self.sub.bat_SOC)
        self.optimiser.update_energy_system()

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
                if self.settings.simulation["use_visualisation"]:
                    self.plot.initial_time = self.sub.initial_time

    def main_loop(self):

        # Updates day counter
        self.update_day_counter()

        # Checks if all subscribers have new values
        all_read = self.sub.battery_read == 1 and self.sub.solar_read == 1 and self.sub.house_read == 1

        # Obtains the current time
        curr_time = self.current_time()

        # True every time step
        curr_data_mod = curr_time % (self.time_step / 60)
        self.data_mod = curr_data_mod < self.mod_thresh or (self.time_step / 60) - curr_data_mod < self.mod_thresh

        # True every control time step
        curr_control_mod = curr_time % (self.control_step / 60)
        self.control_mod = curr_control_mod < self.mod_thresh or (
                    self.control_step / 60) - curr_control_mod < self.mod_thresh

        # True on every optimiser time step
        curr_opt_mod = curr_time % (self.opt_step / 60)
        self.opt_mod = curr_opt_mod < self.mod_thresh or (
                    self.opt_step / 60) - curr_opt_mod < self.mod_thresh

        if all_read:

            # Applies Control if necessary
            self.apply_control()

            # Resets Subscriber Read Values
            self.sub.battery_read = 0
            self.sub.solar_read = 0
            self.sub.house_read = 0

    def apply_control(self):

        # Obtains current house time
        if bool(self.sub.data_store["house_time"]) is False:
            curr_time = 0
        else:
            curr_time = self.sub.data_store["house_time"][-1]

        # Calculates new grid value
        self.pub.set_grid(self.sub.solar_power, self.sub.house_power)

        # PV Self Consumption Control
        if self.settings.control["pv_self_cons"]:
            if self.control_mod and curr_time != self.prev_house_control:

                # Applies Control
                self.pub.non_optimiser_control(self.sub.bat_SOC)
                self.prev_house_control = curr_time

        # Optimiser Control
        if self.settings.control["optimiser"] and len(self.load) == (60 / self.time_step) * 24:
            if self.opt_mod and curr_time != self.prev_house_opt:
                # Applies Control
                self.data_skip = self.sub.house_num
                self.optimiser.optimise()
                self.power = list(self.optimiser.return_battery_power())
                self.data_skip = self.sub.house_num - self.data_skip
                self.optimiser_index = 0 + self.data_skip
                self.prev_house_opt = curr_time

        # Data Time Step
        if self.data_mod and curr_time != self.prev_house_data:

            # Print Outputs
            print('Optimiser skipped ' + str(self.data_skip) + ' time steps')
            print('Total PV system money saved = $' + str(round(self.pub.house_import - self.pub.savings, 2)))
            print('Additional money saved by battery = $' + str(round(self.pub.sol_savings - self.pub.savings, 2)))
            print('Total cost of load import = $' + str(round(self.pub.house_import, 2)))
            print('Total load energy = ' + str(round(self.house_energy, 2)) + ' kWh')
            print('Total solar energy = ' + str(round(self.solar_energy, 2)) + ' kWh')
            print('Day counter = ' + str(self.sub.day_count))

            # Sets new power value
            if self.settings.control["pv_self_cons"] and self.settings.control["optimiser"]:
                if bool(self.power):
                    opt_power = self.power[self.optimiser_index] * 1000 * (60 / self.time_step)
                    opt_power = self.power_scale * opt_power + (1 - self.power_scale) * self.pub.bat_power
                    self.power_filter.step(0, opt_power)
                    self.pub.set_power(self.power_filter.current_state())
                else:
                    self.pub.set_power(0)
            if self.settings.control["optimiser"] and self.settings.control["pv_self_cons"] is False:
                if bool(self.power):
                    opt_power = self.power[self.optimiser_index] * 1000 * (60 / self.time_step)
                    self.pub.set_power(opt_power)
                else:
                    self.pub.set_power(0)

            # Updates optimiser index and 24 hour data
            if self.settings.control["optimiser"]:
                self.optimiser_index += 1
            self.update_24_data(self.data_skip)
            self.prev_house_data = curr_time

            # Updates data stores and optimiser skips
            self.pub.update_data_store("grid", self.pub.grid, self.current_time())
            self.pub.update_data_store("bat", self.pub.bat_power, self.current_time())
            self.data_skip = 0

            # Calculates Savings
            self.calculate_savings()

            # Publishes power
            self.pub.publish_power()


if __name__ == '__main__':

    # Reads settings configuration file and starts drivers
    settings = Settings()
    control = ControlSystem(settings)
    drivers = SunSpecDriver(settings)

    # CONNECTION LOOP
    print('Connecting')
    control.connection_loop()

    # MAIN LOOP
    print('Starting Control System')
    while True:

        # Control System Main Loop
        control.main_loop()

        # Data Visualisation
        if settings.simulation["use_visualisation"]:
            control.plot.update_erase_index(control.sub.data_store["house_time"],
                                            control.sub.soc_num,
                                            control.sub.solar_num,
                                            control.sub.house_num,
                                            control.pub.grid_num,
                                            control.pub.bat_num)

            control.plot.update_plot(control.sub.data_store, control.pub.data_store)

        # Testing Break Condition
        if control.sub.day_count == 7:
            plt.savefig(r"C:\Users\Owner\PycharmProjects\batterycontrolsystem\Test Results\Data Set 3\15kWh_PVSC.png")
            break



