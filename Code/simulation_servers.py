# TCP Modbus server and parts of code from open source: 'uModbus'

"""
TCP function codes:
    3 = read holding registers
    4 = read input registers
    6 = write single holding register
    16 = write multiple holding registers
"""

import csv
import logging
import threading
from collections import defaultdict
from socketserver import TCPServer

import numpy as np
from umodbus import conf
from umodbus.server.tcp import RequestHandler, get_server
from umodbus.utils import log_to_stream


class Battery:
    def __init__(self, battery_data):
        # Initialises Data Store
        self.data_store = defaultdict(int)

        # Enable values to be signed
        conf.SIGNED_VALUES = False

        # Add Stream Handler to Logger
        log_to_stream(level=logging.DEBUG)

        # Creates TCP Server
        TCPServer.allow_reuse_address = True
        self.app = get_server(TCPServer, ('127.0.0.1', 8080), RequestHandler)

        # Server read function
        @self.app.route(slave_ids=[1], function_codes=[3, 4], addresses=list(range(0, 34)))
        def read_data_store(slave_id, function_code, address):
            return self.data_store[address]

        # Server write function
        @self.app.route(slave_ids=[1], function_codes=[6, 16], addresses=list(range(0, 34)))
        def write_data_store(slave_id, function_code, address, value):
            self.data_store[address] = value
            if address == 3:
                self.predict_soc(value)

        # Starting server in background thread
        self.thread = threading.Thread(target=self._thread)
        self.thread.start()

        # Sets Initial Values
        self.initial_soc = 0
        self.SOC = self.initial_soc
        self.dt = 24/len(battery_data)  # Hours
        self.bat_cap = np.trapz(battery_data[0:int(len(battery_data)/2)], dx=self.dt)  # Wh

        self.data_store[0] = 803
        self.data_store[1] = 16
        self.data_store[19] = self.initial_soc

    def set_value(self, new_soc):
        self.SOC = new_soc
        self.data_store[19] = int(new_soc)

    def return_value(self):
        return self.SOC

    def predict_soc(self, power):
        old_soc = self.return_value()

        new_soc = old_soc + ((power/self.bat_cap)*self.dt)*100

        if new_soc > 100:
            print('SOC at 100%')
            new_soc = 100

        elif new_soc < 0:
            print('SOC at 0%')
            new_soc = 0

        self.set_value(new_soc)

    def _thread(self):
        try:
            print('starting battery server')
            self.app.serve_forever()
        finally:
            self.app.shutdown()
            self.app.server_close()
            self.thread = None


class Solar:
    def __init__(self, solar_data):

        # Assigns Initial Data
        self.solar_data = solar_data

        # Initialises Data Store
        self.data_store = defaultdict(int)

        # Enable values to be signed
        conf.SIGNED_VALUES = True

        # Add Stream Handler to Logger
        log_to_stream(level=logging.DEBUG)

        # Creates TCP Server
        TCPServer.allow_reuse_address = True
        self.app = get_server(TCPServer, ('127.0.0.1', 8081), RequestHandler)

        # Sets Initial Value
        self.data_count = 0

        # Server read function
        @self.app.route(slave_ids=[1], function_codes=[3, 4], addresses=list(range(0, 1)))
        def read_data_store(slave_id, function_code, address):
            self.data_store[0] = self.solar_data[self.data_count]
            if self.data_count == len(self.solar_data) - 1:
                self.data_count = 0
            else:
                self.data_count += 1
            return self.data_store[address]

        # Starting server in background thread
        self.thread = threading.Thread(target=self._thread)
        self.thread.start()

    def _thread(self):
        try:
            print('starting solar server')
            self.app.serve_forever()
        finally:
            self.app.shutdown()
            self.app.server_close()
            self.thread = None


class House:
    def __init__(self, house_data):

        # Assigns Initial Data
        self.house_data = house_data
        # Initialises Data Store
        self.data_store = defaultdict(int)

        # Enable values to be signed
        conf.SIGNED_VALUES = True

        # Add Stream Handler to Logger
        log_to_stream(level=logging.DEBUG)

        # Creates TCP Server
        TCPServer.allow_reuse_address = True
        self.app = get_server(TCPServer, ('127.0.0.1', 8082), RequestHandler)

        # Sets Initial Value
        self.data_count = 0

        # Server read function
        @self.app.route(slave_ids=[1], function_codes=[3, 4], addresses=list(range(0, 1)))
        def read_data_store(slave_id, function_code, address):
            self.data_store[0] = self.house_data[self.data_count]
            if self.data_count == len(self.house_data) - 1:
                self.data_count = 0
            else:
                self.data_count += 1
            return self.data_store[address]

        # Starting server in background thread
        self.thread = threading.Thread(target=self._thread)
        self.thread.start()

    def _thread(self):
        try:
            print('starting house server')
            self.app.serve_forever()
        finally:
            self.app.shutdown()
            self.app.server_close()
            self.thread = None


class Servers:
    def __init__(self):

        # Setting up Data
        self.battery_data = []
        self.solar_data = []
        self.house_data = []

        # Servers Variables
        self.battery = None
        self.solar = None
        self.house = None

        # Reading CSV File
        with open('one_day_export.csv', mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                self.battery_data.append(int(float(row["abatteryp"]) * 1000))
                self.solar_data.append(int(float(row["asolarp"]) * 1000))
                self.house_data.append(int(float(row["aloadp"]) * 1000))

    def start(self):
        self.battery = Battery(self.battery_data)
        self.solar = Solar(self.solar_data)
        self.house = House(self.house_data)


if __name__ == '__main__':

    # Starting Server Simulations
    servers = Servers()
    servers.start()

