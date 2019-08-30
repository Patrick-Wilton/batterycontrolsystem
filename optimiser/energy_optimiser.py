from pyomo.opt import SolverFactory
import pyomo.environ as en
import numpy as np
import pandas as pd

####################################################################

# Define some useful container objects to define the optimisation objectives

class OptimiserObjective(object):
    ConnectionPointCost = 1
    ConnectionPointEnergy = 2
    ThroughputCost = 3
    Throughput = 4
    GreedySolarCharging = 5
    GreedyLoadDischarging = 6
    EqualStorageActions = 7
    ConnectionPointPeakPower = 8
    ConnectionPointQuantisedPeak = 9
    PiecewiseLinear = 10
    
class OptimiserObjectiveSet(object):
    FinancialOptimisation = [#OptimiserObjective.ConnectionPointCost,
                             OptimiserObjective.GreedySolarCharging,
                             OptimiserObjective.ThroughputCost,
                             OptimiserObjective.EqualStorageActions]

    EnergyOptimisation = [OptimiserObjective.ConnectionPointEnergy,
                          OptimiserObjective.GreedySolarCharging,
                          OptimiserObjective.GreedyLoadDischarging,
                          OptimiserObjective.Throughput,
                          OptimiserObjective.EqualStorageActions]

    PeakOptimisation = [OptimiserObjective.ConnectionPointPeakPower,
                        OptimiserObjective.EqualStorageActions]

    QuantisedPeakOptimisation = [OptimiserObjective.ConnectionPointQuantisedPeak]

    DispatchOptimisation = [OptimiserObjective.PiecewiseLinear] + FinancialOptimisation

# Define some useful constants
minutes_per_hour = 60.0

####################################################################

class EnergyOptimiser(object):
    
    def __init__(self, interval_duration, number_of_intervals, energy_system, objective):
        self.interval_duration = interval_duration  # The duration (in minutes) of each of the intervals being optimised over
        self.number_of_intervals = number_of_intervals
        self.energy_system = energy_system

        # This must be configured correctly on the host machine
        self.optimiser_engine = "cplex"  # 'gurobi' / 'glpk'
        # self.optimiser_engine_executable = "/Applications/CPLEX_Studio128/cplex/bin/x86-64_osx/cplex"
        # self.optimiser_engine_executable = "/opt/ibm/ILOG/CPLEX_Studio128/cplex/bin/x86-64_linux/cplex"
        self.optimiser_engine_executable = r"C:\Program Files\IBM\ILOG\CPLEX_Studio128\cplex\bin\x64_win64\cplex"

        self.use_bool_vars = False
        self.use_piecewise_segments = True  # Defined for a future implementation of linear piecewise segments
        self.bigM = 5000000

        # This value has been arbitrarily chosen
        # A better understanding of the sensitivity of this value may be advantageous
        self.smallM = 0.0001

        self.objectives = objective
        self.build_model()
        self.apply_constraints()
        self.build_objective()
        self.optimise()

        
    def build_model(self):
        # Set up the Pyomo model
        self.model = en.ConcreteModel()

        # We use RangeSet to create a index for each of the time
        # periods that we will optimise within.
        self.model.Time = en.RangeSet(0, self.number_of_intervals - 1)

        # Convert the data into the right format for the optimiser
        load = self.energy_system.load.load
        generation = self.energy_system.pv.pv
        net_load = load + generation

        # split net load into import and export
        connection_point_import = np.copy(net_load)
        connection_point_export = np.copy(net_load)
        for j, e in enumerate(net_load):
            if e >= 0:
                connection_point_export[j] = 0
            else:
                connection_point_import[j] = 0
        import_load_dct = dict(enumerate(connection_point_import))
        export_load_dct = dict(enumerate(connection_point_export))

        #### Initialise the optimisation variables (all indexed by Time) ####

        # The state of charge of the battery
        self.model.storage_state_of_charge = en.Var(self.model.Time,
                                                    bounds=(0, self.energy_system.energy_storage.capacity),
                                                    initialize=0)

        # The increase in energy storage state of charge at each time step
        self.model.storage_charge_total = en.Var(self.model.Time, initialize=0)

        # The decrease in energy storage state of charge at each time step
        self.model.storage_discharge_total = en.Var(self.model.Time, initialize=0)

        # Increase in battery SoC from the Grid
        self.model.storage_charge_grid = en.Var(self.model.Time,
                                                bounds=(0, self.energy_system.energy_storage.charging_power_limit *
                                                        (self.interval_duration / minutes_per_hour)),
                                                initialize=0)

        # Increase in battery SoC from PV Generation
        self.model.storage_charge_generation = en.Var(self.model.Time,
                                                      bounds=(0, self.energy_system.energy_storage.charging_power_limit *
                                                                 (self.interval_duration / minutes_per_hour)),
                                                      initialize=0)

        # Satisfying local demand from the battery
        self.model.storage_discharge_load = en.Var(self.model.Time,
                                                   bounds=(self.energy_system.energy_storage.discharging_power_limit *
                                                           (self.interval_duration / minutes_per_hour), 0),
                                                   initialize=0)

        # Exporting to the grid from the battery
        self.model.storage_discharge_grid = en.Var(self.model.Time,
                                                   bounds=(self.energy_system.energy_storage.discharging_power_limit *
                                                           (self.interval_duration / minutes_per_hour), 0),
                                                   initialize=0)

        # Net import from the grid
        self.model.net_import = en.Var(self.model.Time, initialize=import_load_dct)

        # Net export to the grid
        self.model.net_export = en.Var(self.model.Time, initialize=export_load_dct)


        #### Boolean variables (again indexed by Time) ####

        # These may not be necessary so provide a binary flag for turning them off

        if self.use_bool_vars:
            # Is the battery charging in a given time interval
            self.model.is_charging = en.Var(self.model.Time, within=en.Boolean)
            # Is the battery discharging in a given time interval
            self.model.is_discharging = en.Var(self.model.Time, within=en.Boolean, initialize=0)


        #### Battery Parameters ####

        # The battery charging efficiency
        self.model.etaChg = en.Param(initialize=self.energy_system.energy_storage.charging_efficiency)
        # The battery discharging efficiency
        self.model.etaDisChg = en.Param(initialize=self.energy_system.energy_storage.discharging_efficiency)
        # The battery charge power limit
        self.model.ChargingLimit = en.Param(
            initialize=self.energy_system.energy_storage.charging_power_limit * (self.interval_duration / minutes_per_hour))
        # The battery discharge power limit
        self.model.DischargingLimit = en.Param(
            initialize=self.energy_system.energy_storage.discharging_power_limit * (self.interval_duration / minutes_per_hour))

        #### Initial Load / Solar Profile Parameters ####
        # The local energy consumption
        self.model.local_energy_consumption = en.Param(self.model.Time, initialize=import_load_dct)
        # The local energy generation
        self.model.local_energy_generation = en.Param(self.model.Time, initialize=export_load_dct)


        #### Tariffs and Financial Incentives ####
        # The import tariff per kWh
        self.model.priceBuy = en.Param(self.model.Time, initialize=self.energy_system.tariff.import_tariff)
        # The export tariff per kWh
        self.model.priceSell = en.Param(self.model.Time, initialize=self.energy_system.tariff.export_tariff)
        # The throughput cost for the energy storage
        self.model.throughput_cost = en.Param(initialize=self.energy_system.energy_storage.throughput_cost)

        #### Bias Values ####
        # A small fudge factor for reducing the size of the solution set and
        # achieving a unique optimisation solution
        self.model.scale_func = en.Param(initialize=self.smallM)
        # A bigM value for integer optimisation
        self.model.bigM = en.Param(initialize=self.bigM)

        #### Connection Point Peak Power ####
        self.model.peak_connection_point_import_power = en.Var(within=en.NonNegativeReals)
        self.model.peak_connection_point_export_power = en.Var(within=en.NonNegativeReals)

        def peak_connection_point_import(model, interval):
            return model.peak_connection_point_import_power >= model.net_import[interval]

        def peak_connection_point_export(model, interval):
            return model.peak_connection_point_export_power >= -model.net_export[interval]

        self.model.peak_connection_point_import_constraint = en.Constraint(self.model.Time, rule=peak_connection_point_import)
        self.model.peak_connection_point_export_constraint = en.Constraint(self.model.Time, rule=peak_connection_point_export)


        #### Piecewise Linear Segments (To be fully implemented later) ####
        '''if self.use_piecewise_segments:
            # The turning points for the piecewise linear segments
            self.model.turning_point_one_ramp = en.Var(self.model.Time, within=en.Boolean, initialize=0)
            self.model.turning_point_two_ramp = en.Var(self.model.Time, within=en.Boolean, initialize=0)
            lims_one = [None] * (len(net) - 1)  # ToDo - Fix this indexing
            lims_two = [None] * (len(net) - 1)  # ToDo - Fix this indexing

            ind = self.energy_system.dispatch.linear_ramp[0]
            lim = self.energy_system.dispatch.linear_ramp[1]
            for i, l in zip(ind, lim):
                lims_one[i] = l[0]
                lims_two[i] = l[1]

            lim_dct_one = dict(enumerate(lims_one))
            self.model.limits_one = en.Param(self.model.Time, initialize=lim_dct_one)

            lim_dct_two = dict(enumerate(lims_two))
            self.model.limits_two = en.Param(self.model.Time, initialize=lim_dct_two)

            self.model.my_set = en.Set(initialize=ind)
            def B1(m, s):
                return m.limits_one[s] <= m.storage_charge_total[s] + m.storage_discharge_total[s] + self.bigM * (1 - m.turning_point_one_ramp[s])

            def B2(m, s):
                return m.limits_one[s] >= m.storage_charge_total[s] + m.storage_discharge_total[s] - self.bigM * m.turning_point_one_ramp[s]

            self.model.B1 = en.Constraint(self.model.my_set, rule=B1)
            self.model.B2 = en.Constraint(self.model.my_set, rule=B2)

            def B3(m, s):
                return m.limits_two[s] <= m.storage_charge_total[s] + m.storage_discharge_total[s] + self.bigM * (1 - m.turning_point_two_ramp[s])

            def B4(m, s):
                return m.limits_two[s] >= m.storage_charge_total[s] + m.storage_discharge_total[s] - self.bigM * m.turning_point_two_ramp[s]

            self.model.B3 = en.Constraint(self.model.my_set, rule=B3)
            self.model.B4 = en.Constraint(self.model.my_set, rule=B4)'''


    def apply_constraints(self):

        # Calculate the increased state of charge of the energy storage from the
        # imported energy and locally generated energy. We ensure that the
        # storage charging efficiency is taken into account.
        def storage_charge_behaviour(model, time_interval):
            return model.storage_charge_grid[time_interval] + model.storage_charge_generation[time_interval] \
                   == model.storage_charge_total[time_interval] / model.etaChg

        # Calculate the decreased state of charge of the energy storage from the
        # exported energy and locally consumed energy. We ensure that the
        # storage discharging efficiency is taken into account.
        def storage_discharge_behaviour(model, time_interval):
            return model.storage_discharge_load[time_interval] + model.storage_discharge_grid[time_interval] \
                   == model.storage_discharge_total[time_interval] * model.etaDisChg

        # Enforce the charging rate limit
        def storage_charge_rate_limit(model, time_interval):
            return (model.storage_charge_grid[time_interval] + model.storage_charge_generation[
                time_interval]) <= model.ChargingLimit

        # Enforce the discharge rate limit
        def storage_discharge_rate_limit(model, time_interval):
            return (model.storage_discharge_load[time_interval] + model.storage_discharge_grid[
                time_interval]) >= model.DischargingLimit

        # Add the constraints to the optimisation model
        self.model.storage_charge_behaviour_constraint = en.Constraint(self.model.Time, rule=storage_charge_behaviour)
        self.model.storage_discharge_behaviour_constraint = en.Constraint(self.model.Time, rule=storage_discharge_behaviour)
        self.model.storage_charge_rate_limit_constraint = en.Constraint(self.model.Time, rule=storage_charge_rate_limit)
        self.model.storage_discharge_rate_limit_constraint = en.Constraint(self.model.Time, rule=storage_discharge_rate_limit)

        # Enforce the limits of charging the energy storage from locally generated energy
        def storage_solar_charging_behaviour(model, time_interval):
            return model.storage_charge_generation[time_interval] <= -model.local_energy_generation[time_interval]

        # Enforce the limits of discharging the energy storage to satisfy local demand
        def storage_discharging_consumption_behaviour(model, time_interval):
            return model.storage_discharge_load[time_interval] >= -model.local_energy_consumption[time_interval]

        # Add the constraints to the optimisation model
        self.model.solar_charging_behaviour_constraint = en.Constraint(self.model.Time, rule=storage_solar_charging_behaviour)
        self.model.local_discharge_behaviour_constraint = en.Constraint(self.model.Time, rule=storage_discharging_consumption_behaviour)

        # Calculate the state of charge of the battery in each time interval
        initial_state_of_charge = self.energy_system.energy_storage.initial_state_of_charge

        def SOC_rule(model, time_interval):
            if time_interval == 0:
                return model.storage_state_of_charge[time_interval] \
                       == initial_state_of_charge + model.storage_charge_total[time_interval] + \
                       model.storage_discharge_total[
                           time_interval]
            else:
                return model.storage_state_of_charge[time_interval] \
                       == model.storage_state_of_charge[time_interval - 1] + model.storage_charge_total[time_interval] + \
                       model.storage_discharge_total[time_interval]

        # Calculate the net energy import
        def net_connection_point_import(model, time_interval):
            return model.net_import[time_interval] == model.local_energy_consumption[time_interval] + \
                   model.storage_charge_grid[time_interval] + model.storage_discharge_load[time_interval]

        # calculate the net energy export
        def net_connection_point_export(model, time_interval):
            return model.net_export[time_interval] == model.local_energy_generation[time_interval] + \
                   model.storage_charge_generation[time_interval] + model.storage_discharge_grid[time_interval]

        # Add the constraints to the optimisation model
        self.model.Batt_SOC = en.Constraint(self.model.Time, rule=SOC_rule)
        self.model.net_import_constraint = en.Constraint(self.model.Time, rule=net_connection_point_import)
        self.model.net_export_constraint = en.Constraint(self.model.Time, rule=net_connection_point_export)

        # Use bigM formulation to ensure that the battery is only charging or discharging in each time interval
        if self.use_bool_vars:

            # If the battery is charging then the charge energy is bounded from below by -bigM
            # If the battery is discharging the charge energy is bounded from below by zero
            def bool_cd_rule_one(model, time_interval):
                return model.storage_charge_total[time_interval] >= -self.model.bigM * model.is_charging[time_interval]

            # If the battery is charging then the charge energy is bounded from above by bigM
            # If the battery is discharging the charge energy is bounded from above by zero
            def bool_cd_rule_two(model, time_interval):
                return model.storage_charge_total[time_interval] <= self.model.bigM * (1 - model.is_discharging[time_interval])

            # If the battery is charging then the discharge energy is bounded from above by zero
            # If the battery is discharging the discharge energy is bounded from above by bigM
            def bool_cd_rule_three(model, time_interval):
                return model.storage_discharge_total[time_interval] <= self.model.bigM * model.is_discharging[time_interval]

            # If the battery is charging then the discharge energy is bounded from below by zero
            # If the battery is discharging the discharge energy is bounded from below by -bigM
            def bool_cd_rule_four(model, time_interval):
                return model.storage_discharge_total[time_interval] >= -self.model.bigM * (1 - model.is_charging[time_interval])

            # The battery can only be charging or discharging
            def bool_cd_rule_five(model, time_interval):
                return model.is_charging[time_interval] + model.is_discharging[time_interval] == 1

            # Add the constraints to the optimisation model
            self.model.bcdr_one = en.Constraint(self.model.Time, rule=bool_cd_rule_one)
            self.model.bcdr_two = en.Constraint(self.model.Time, rule=bool_cd_rule_two)
            self.model.bcdr_three = en.Constraint(self.model.Time, rule=bool_cd_rule_three)
            self.model.bcdr_four = en.Constraint(self.model.Time, rule=bool_cd_rule_four)
            self.model.bcdr_five = en.Constraint(self.model.Time, rule=bool_cd_rule_five)


    def build_objective(self):
        # Build the objective function ready for optimisation
        objective = 0

        if OptimiserObjective.ConnectionPointCost in self.objectives:
            # Connection point cost
            objective += sum(self.model.priceBuy[i] * self.model.net_import[i] +  # The cost of purchasing energy
                             self.model.priceSell[i] * self.model.net_export[i]  # The value of selling energy
                             for i in self.model.Time)

        if OptimiserObjective.ConnectionPointEnergy in self.objectives:
            # The amount of energy crossing the meter boundary
            objective += sum((-self.model.net_export[i] + self.model.net_import[i])
                             for i in self.model.Time)

        if OptimiserObjective.ThroughputCost in self.objectives:
            # Throughput cost of using energy storage - we attribute half the cost to charging and half to discharging
            objective += sum(self.model.storage_charge_total[i] - self.model.storage_discharge_total[i]
                             for i in self.model.Time) * self.model.throughput_cost / 2.0

        if OptimiserObjective.Throughput in self.objectives:
            # Throughput of using energy storage - it mirrors the throughput cost above
            objective += sum(self.model.storage_charge_total[i] - self.model.storage_discharge_total[i]
                             for i in self.model.Time) * self.model.scale_func

        if OptimiserObjective.GreedySolarCharging in self.objectives:
            # Greedy Solar - Favour charging fully from solar in earlier intervals
            objective += sum(-self.model.net_export[i]
                             * 1 / self.number_of_intervals
                             * (1 - i / self.number_of_intervals)
                             for i in self.model.Time)

        if OptimiserObjective.GreedyLoadDischarging in self.objectives:
            # Greedy Load Discharging - Favour satisfying all load from the storage in earlier intervals
            objective += sum(self.model.net_import[i]
                             * 1 / self.number_of_intervals
                             * (1 - i / self.number_of_intervals)
                             for i in self.model.Time)

        if OptimiserObjective.EqualStorageActions in self.objectives:
            # ToDo - Which is the better implementation?
            objective += sum((self.model.storage_charge_grid[i] * self.model.storage_charge_grid[i]) +
                             (self.model.storage_charge_generation[i] * self.model.storage_charge_generation[i]) +
                             (self.model.storage_discharge_grid[i] * self.model.storage_discharge_grid[i]) +
                             (self.model.storage_discharge_load[i] * self.model.storage_discharge_load[i])
                             for i in self.model.Time) * self.model.scale_func

            '''objective += sum(self.model.storage_charge_total[i] * self.model.storage_charge_total[i] +
                             self.model.storage_discharge_total[i] * self.model.storage_discharge_total[i]
                             for i in self.model.Time) * self.model.scale_func'''

        if OptimiserObjective.ConnectionPointPeakPower in self.objectives:
            # ToDo - More work is needed to convert this into a demand tariff objective (i.e. a cost etc.)
            objective += self.model.peak_connection_point_import_power + self.model.peak_connection_point_export_power


        if OptimiserObjective.ConnectionPointQuantisedPeak in self.objectives:
            # ToDo - What is this objective function? Quantises the Connection point?
            objective += sum(self.model.net_export[i] * self.model.net_export[i] +
                             self.model.net_import[i] * self.model.net_import[i]
                             for i in self.model.Time)

        '''if OptimiserObjective.PiecewiseLinear in self.objectives: # ToDo - Fix this implementation to make it complete
            for i in self.energy_system.dispatch.linear_ramp[0]:
                objective += -1 * (self.model.storage_charge_total[i] + self.model.storage_discharge_total[i]) * (
                        1 - self.model.turning_point_two_ramp[i])'''

        def objective_function(model):
            return objective

        self.model.total_cost = en.Objective(rule=objective_function, sense=en.minimize)


    def optimise(self):
        # set the path to the solver
        if self.optimiser_engine == 'cplex':
            opt = SolverFactory(self.optimiser_engine, executable=self.optimiser_engine_executable)
        else:
            opt = SolverFactory(self.optimiser_engine)

        # Solve the optimisation
        self.results = opt.solve(self.model)