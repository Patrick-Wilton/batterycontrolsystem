
import csv

import numpy as np
from pyomo.core import Var

from optimiser.energy_optimiser import EnergyOptimiser, OptimiserObjectiveSet
from optimiser.models import EnergyStorage, EnergySystem, Load, PV, Tariff


class InitialPrediction:
    def __init__(self, config_settings):

        # Reads settings config file
        self.settings = config_settings

        # Creates data stores
        self.solar_data = list()
        self.house_data = list()

        # Data File Names
        file_name = self.settings.simulation["data_file_name"]
        solar_name = self.settings.simulation["solar_row_name"]
        house_name = self.settings.simulation["house_row_name"]

        # Reading CSV File
        with open(file_name, mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                self.solar_data.append(float(row[solar_name]))
                self.house_data.append(float(row[house_name]))


class Optimiser:
    def __init__(self, config_settings):

        # Reads settings config file
        self.settings = config_settings

        # Defines battery model
        self.battery = EnergyStorage(max_capacity=self.settings.battery["max_capacity"],
                                     depth_of_discharge_limit=self.settings.battery["DOD_limit"],
                                     charging_power_limit=self.settings.battery["charging_power_limit"],
                                     discharging_power_limit=self.settings.battery["discharging_power_limit"],
                                     charging_efficiency=self.settings.battery["charging_efficiency"],
                                     discharging_efficiency=self.settings.battery["discharging_efficiency"],
                                     throughput_cost=self.settings.battery["throughput_cost"],
                                     initial_state_of_charge=self.settings.battery["initial_SOC"])

        # Creates Energy System and Model
        self.energy_system = EnergySystem()
        self.energy_system.add_energy_storage(self.battery)
        self.model = None
        self.num_output_variables = 12

        # Creates Initial Optimiser Prediction (24 hours of data)
        if self.settings.control["initial_optimiser_prediction"]:
            self.initial_data = InitialPrediction(self.settings)
            self.load = np.array(self.initial_data.house_data)
            self.pv = np.array(self.initial_data.solar_data)
        else:
            self.load = np.array(list())
            self.pv = np.array(list())

        self.load /= 12
        self.pv /= 12

        # Creates load and solar profiles
        self.load_profile = Load()
        self.pv_profile = PV()

        # Creates Tariffs
        self.time_step = 15 #self.settings.control["data_time_step"]
        self.total_steps = (60 / self.time_step) * 24 * 3

        import_tariff = np.array(([0.1] * 84 + [0.3] * 24 + [0.2] * 96 + [0.3] * 48 + [0.1] * 36))  # MAGIC NUMS
        export_tariff = np.array(([0.0] * 288))  # MAGIC NUMS

        self.import_tariff = dict(enumerate(import_tariff))
        self.export_tariff = dict(enumerate(export_tariff))

        self.tariff_profile = Tariff()

        # Predefine Output Variables
        self.output_vars = np.zeros((self.num_output_variables, len(export_tariff)))

        # Sets Optimiser Objective
        self.objective = None
        self.set_objective()

        # Performs Initial Profile and Energy System Set
        self.update_profiles(self.load,
                             self.pv,
                             self.import_tariff,
                             self.export_tariff,
                             self.settings.battery["initial_SOC"])
        self.update_energy_system()

    def set_objective(self):
        if self.settings.control["objective"] == "Financial":
            self.objective = OptimiserObjectiveSet.FinancialOptimisation
        elif self.settings.control["objective"] == "Energy":
            self.objective = OptimiserObjectiveSet.EnergyOptimisation
        elif self.settings.control["objective"] == "Peak":
            self.objective = OptimiserObjectiveSet.PeakOptimisation
        elif self.settings.control["objective"] == "QuantisedPeak":
            self.objective = OptimiserObjectiveSet.QuantisedPeakOptimisation
        elif self.settings.control["objective"] == "Dispatch":
            self.objective = OptimiserObjectiveSet.DispatchOptimisation
        else:
            print('not a setting')

    def update_profiles(self, load, pv, imp, exp, soc):
        self.load_profile.add_load_profile(load)
        self.pv_profile.add_pv_profile(pv)
        self.tariff_profile.add_tariff_profile_import(imp)
        self.tariff_profile.add_tariff_profile_export(exp)
        self.battery.initial_state_of_charge = soc / (100 / self.battery.max_capacity)

    def update_energy_system(self):
        self.energy_system.add_load(self.load_profile)
        self.energy_system.add_pv(self.pv_profile)
        self.energy_system.add_tariff(self.tariff_profile)

    def optimise(self):

        optimiser = EnergyOptimiser(self.time_step, self.total_steps, self.energy_system, self.objective)
        self.model = optimiser.model

    def return_battery_power(self):

        # Extract Optimiser Results
        j = 0
        for v in self.model.component_objects(Var, active=True):
            var_object = getattr(self.model, str(v))
            for index in var_object:
                self.output_vars[j, index] = var_object[index].value
            j += 1
            if j >= self.num_output_variables:
                break

        # Calculate total new battery power over 24 hours
        storage_energy_delta = self.output_vars[3] + self.output_vars[4] + self.output_vars[5] + self.output_vars[6]
        return storage_energy_delta




