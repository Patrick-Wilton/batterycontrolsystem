

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pyomo.core import Var

# Since using the data-driven data for the testing, use their battery class
import sys
sys.path.append("../")
from optimiser.models import EnergyStorage, EnergySystem, Load, PV, Tariff, DispatchRequest
from optimiser.energy_optimiser import EnergyOptimiser, OptimiserObjectiveSet


# set up seaborn the way you like
sns.set_style({'axes.linewidth': 1, 'axes.edgecolor': 'black', 'xtick.direction': \
    'out', 'xtick.major.size': 4.0, 'ytick.direction': 'out', 'ytick.major.size': 4.0, \
               'axes.facecolor': 'white', 'grid.color': '.8', 'grid.linestyle': u'-', 'grid.linewidth': 0.5})

############################ Define an Example Optimisation Problem ########################################

battery = EnergyStorage(max_capacity=15.0,
                        depth_of_discharge_limit=0,
                        charging_power_limit=5.0,
                        discharging_power_limit=-5.0,
                        charging_efficiency=1,
                        discharging_efficiency=1,
                        throughput_cost=0.018)

# The load and pv arrays below are in kwh consumed per 15 minutes
test_load = np.array([2.13, 2.09, 2.3, 2.11, 2.2, 2.23, 2.2, 2.15, 2.02, 2.19, 2.19, 2.19, 2.12, 2.15, 2.25, 2.12, 2.21, 2.16,
                      2.26, 2.13, 2.08, 2.15, 2.42, 2.02, 2.3, 2.26, 2.35, 2.55, 3.23, 2.98, 3.49, 3.5, 3.12, 3.52, 3.94, 3.55,
                      3.99, 3.71, 3.38, 3.76, 3.71, 3.78, 3.29, 3.65, 3.61, 3.75, 3.38, 3.66, 3.56, 3.69, 3.3, 3.61, 3.71, 3.82,
                      3.17, 3.69, 3.74, 3.86, 3.57, 3.55, 3.75, 3.6, 3.67, 3.48, 3.51, 3.46, 3.19, 3.38, 3.19, 3.38, 3.04, 3.12,
                      2.91, 3.11, 3.13, 2.77, 2.24, 2.54, 2.24, 2.24, 2.09, 2.33, 2.17, 2.16, 1.97, 2.16, 2.21, 2.18, 2.01, 2.16,
                      2.19, 2.11, 2.17, 2.13, 2.05, 2.19])

#test_load = np.concatenate([test_load] * 3)
test_load = np.array([0.0, 1.0] * 144)


# test_pv = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.23, 0.52,
#                     0.74, 0.71, 0.63, 0.68, 0.97, 0.01, 0.52, 0.83, 0.83, 0.79, 1.22, 1.36, 1.27, 1.42, 1.97, 2.56, 2.91, 3.24,
#                     3.8, 4.3, 4.62, 4.84, 4.6, 4.17, 3.77, 3.76, 3.38, 2.64, 1.96, 1.76, 1.85, 2.4, 3.82, 5.13, 4.97, 5.02, 5.43,
#                     5.32, 3.56, 1.75, 1.43, 1.65, 1.69, 2.3, 2.71, 2.41, 2.63, 2.6, 1.9, 0.78, 0.13, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#
# test_pv = np.concatenate([test_pv] * 3)
test_pv = np.array([1.0, 0.0] * 144)

test_pv *= -1  # convert solar generation to negative to match convention.

net_load = test_load + test_pv


# Tariffs are in $ / kwh
# import_tariff = np.array(([0.1] * 28 + [0.3] * 8 + [0.2] * 32 + [0.3] * 16 + [0.1] * 12))
import_tariff = np.array([1.0] * 288)

# import_tariff = np.concatenate([import_tariff] * 3)
export_tariff = np.array(([0.0] * 288))
export_tariff_dct = dict(enumerate(export_tariff))
import_tariff_dct = dict(enumerate(import_tariff))

colors = sns.color_palette()
hrs = np.arange(0, len(test_load)) / 4
fig = plt.figure(figsize=(14, 4))
ax1 = fig.add_subplot(2, 1, 1)
l1, = ax1.plot(hrs, 4 * test_load, color=colors[0])
l2, = ax1.plot(hrs, 4 * test_pv, color=colors[1])
l3, = ax1.plot(hrs, 4 * net_load, color=colors[2])
ax1.set_xlabel('hour'), ax1.set_ylabel('kW')
ax1.legend([l1, l2, l3], ['Load', 'PV', 'Connection Point'], ncol=2)
ax1.set_xlim([0, len(test_load) / 4])
ax2 = fig.add_subplot(2, 1, 2)
l1, = ax2.plot(hrs, import_tariff, color=colors[3])
l2, = ax2.plot(hrs, export_tariff, color=colors[4])
ax2.set_xlabel('hour'), ax2.set_ylabel('price ($/kWh)')
ax2.legend([l1, l2], ['buy price', 'sell price'], ncol=2)
ax2.set_xlim([0, len(test_load) / 4])
fig.tight_layout()

fig.show()

############################ Optimise this Example ########################################

energy_system = EnergySystem()
energy_system.add_energy_storage(battery)

load = Load()
load.add_load_profile(test_load)

pv = PV()
pv.add_pv_profile(test_pv)

tariff = Tariff()
tariff.add_tariff_profile_export(export_tariff_dct)
tariff.add_tariff_profile_import(import_tariff_dct)

energy_system.add_load(load)
energy_system.add_pv(pv)
energy_system.add_tariff(tariff)

# print(energy_system.load.load)
# print(energy_system.pv.pv)

# Dispatch capabilities will be added in a future version
'''dispatch = DispatchRequest()
req = [[4, 7, 12], [(0, 5000), (0, 15000), (0, 28000)]]
req = [[4], [(0, 15000)]]
dispatch.add_dispatch_request_linear_ramp(req)
energy_system.add_dispatch(dispatch)'''

# Invoke the optimiser and optimise
optimiser = EnergyOptimiser(5, 288, energy_system, OptimiserObjectiveSet.PeakOptimisation)




############################ Analyse the Optimisation ########################################
model = optimiser.model

num_ouputvars = 12
outputVars = np.zeros((num_ouputvars, len(export_tariff)))

j = 0
for v in model.component_objects(Var, active=True):
    print(v)
    varobject = getattr(model, str(v))
    for index in varobject:
        outputVars[j, index] = varobject[index].value
    j += 1
    if j >= num_ouputvars:
        break

storage_energy_delta = outputVars[3] + outputVars[4] + outputVars[5] + outputVars[6]
optimised_connection_point_load = outputVars[7] + outputVars[8]
print(outputVars[8])

colors = sns.color_palette()
hrs = np.arange(0, len(test_load)) / 4
fig = plt.figure(figsize=(14, 7))
ax1 = fig.add_subplot(3, 1, 1)
l1, = ax1.plot(hrs, 4 * test_load, color=colors[0])
l2, = ax1.plot(hrs, 4 * test_pv, color=colors[1])
l3, = ax1.plot(hrs, 4 * optimised_connection_point_load, color=colors[2])
l4, = ax1.plot(hrs, 4 * storage_energy_delta, color=colors[3])
ax1.set_xlabel('hour'), ax1.set_ylabel('kW')
ax1.legend([l1, l2, l3, l4], ['Load', 'PV', 'Connection Point', 'Storage'], ncol=3)
ax1.set_xlim([0, len(test_load) / 4])
ax2 = fig.add_subplot(3, 1, 2)
l1, = ax2.plot(hrs, import_tariff, color=colors[3])
l2, = ax2.plot(hrs, export_tariff, color=colors[4])
ax2.set_xlabel('hour'), ax2.set_ylabel('price')
ax2.legend([l1, l2], ['buy price', 'sell price'], ncol=2)
ax2.set_xlim([0, len(test_load) / 4])
ax3 = fig.add_subplot(3, 1, 3)
l1, = ax3.plot(hrs, np.sum(outputVars[3:7, :] * 4, axis=0), color=colors[5])
l2, = ax3.plot(hrs, outputVars[0], color=colors[4])
ax3.set_xlabel('hour'), ax3.set_ylabel('action')
ax3.legend([l1, l2], ['battery action (kW)', 'SOC (kWh)'], ncol=2)
ax3.set_xlim([0, len(test_load) / 4])
fig.tight_layout()
plt.show()