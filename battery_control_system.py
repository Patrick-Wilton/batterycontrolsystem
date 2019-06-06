# TCP Modbus client and parts of code from open source: 'uModbus'

# TODO: Add SunSpec read capabilities

'''
TCP function codes:
    3 = read holding registers
    4 = read input registers
    6 = write single holding register
    16 = write multiple holding registers
'''

# Import Necessary Libraries for TCP client and Battery Server
import csv
import time
import zmq

import numpy as np
from sunspec.core.client import ClientDevice

from simulation_servers import Battery, Solar, House, Timing


class Data:
    def __init__(self):
        self.battery_data = []
        self.solar_data = []
        self.house_data = []
        with open('one_day_export.csv', mode='r') as csv_file:
            self.csv_reader = csv.DictReader(csv_file)
            for row in self.csv_reader:
                self.battery_data.append(int((float(row["abatteryp"])*1000)))
                self.solar_data.append(int(float(row["asolarp"])*1000))
                self.house_data.append(int(float(row["aloadp"])*1000))


if __name__ == '__main__':

    # Initialises Classes
    data = Data()
    battery = Battery(data.battery_data)
    solar = Solar(data.solar_data)
    house = House(data.house_data)
    publisher = Timing()

    battery_client = ClientDevice(device_type='TCP', slave_id=1, ipaddr='localhost', ipport=8080)
    solar_client = ClientDevice(device_type='TCP', slave_id=1, ipaddr='localhost', ipport=8081)
    house_client = ClientDevice(device_type='TCP', slave_id=1, ipaddr='localhost', ipport=8082)

    static_reference = 0

    # Waits for Servers
    time.sleep(0.5)

    # Main for loop
    for dt in range(len(data.battery_data)):
        solar_decode = solar_client.read(0, 1)
        solar_value = np.int16(int.from_bytes(solar_decode, byteorder='big'))
        # print(solar_value)


