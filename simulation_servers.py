# TCP Modbus server and parts of code from open source: 'uModbus'

# TODO: Add Reactive and Real Power
# TODO:

'''
TCP function codes:
    3 = read holding registers
    4 = read input registers
    6 = write single holding register
    16 = write multiple holding registers
'''

import logging
import threading
import time
import zmq
from collections import defaultdict
from socketserver import TCPServer

import numpy as np
from umodbus import conf
from umodbus.server.tcp import RequestHandler, get_server
from umodbus.utils import log_to_stream


class Timing:
    def __init__(self):
        self.init_time = time.time()
        self.time_scale = 8000
        self.total_time = (24/self.time_scale)*3600
        self.end_time = self.init_time + self.total_time
        print((24/self.time_scale)*3600)
        self.pub_interval = 5  # minutes
        self.pub_interval = self.total_time/(24*(60/self.pub_interval))

        self.port = "8090"
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:%s" % self.port)
        print("pub start")
        self.thread = threading.Thread(target=self.publish)
        self.thread.start()

    def publish(self):
        while time.time() < self.end_time:
            topic = 1
            message_data = 2
            self.socket.send_string("%d %d" % (topic, message_data))
            time.sleep(self.pub_interval)
            print("publishing")
        print("DONE")
        self.thread = None


class BatteryServer:
    def __init__(self):
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
        @self.app.route(slave_ids=[1], function_codes=[3, 4, 6, 16], addresses=list(range(0, 34)))
        def read_data_store(slave_id, function_code, address):
            return self.data_store[address]

        # Server write function
        @self.app.route(slave_ids=[1], function_codes=[6, 16], addresses=list(range(0, 34)))
        def write_data_store(slave_id, function_code, address, value):
            self.data_store[address] = value

        # Starting server in background thread
        self.thread = threading.Thread(target=self._thread)
        self.thread.start()

    def _thread(self):
        try:
            print('starting battery server')
            self.app.serve_forever()
        finally:
            self.app.shutdown()
            self.app.server_close()
            self.thread = None


class Battery:
    def __init__(self, battery_data):
        # Server Instance
        self.bat_server = BatteryServer()

        # Sets Initial Values
        self.initial_soc = 0
        self.SOC = 0
        self.dt = 24/len(battery_data)  # Hours
        self.bat_cap = np.trapz(battery_data[0:int(len(battery_data)/2)], dx=self.dt)  # Wh

        self.bat_server.data_store[0] = 803
        self.bat_server.data_store[1] = 16
        self.bat_server.data_store[19] = self.initial_soc
        print('\nbattery initialised')

    def set_value(self, new_soc):
        self.SOC = new_soc
        self.bat_server.data_store[19] = new_soc

    def return_value(self):
        return self.SOC

    def predict_soc(self, power):
        old_soc = self.return_value()
        new_soc = old_soc + (power/self.bat_cap)*self.dt
        new_soc = round(new_soc)

        if new_soc > 100:
            print('SOC at 100%')
            new_soc = 100

        elif new_soc < 0:
            print('SOC at 0%')
            new_soc = 0

        self.set_value(new_soc)
        return new_soc


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
        self.data_store[0] = self.solar_data[0]
        self.data_store[1] = 0
        self.data_count = 0

        # Server read function
        @self.app.route(slave_ids=[1], function_codes=[3, 4, 6, 16], addresses=list(range(0, 1)))
        def read_data_store(slave_id, function_code, address):
            self.data_store[1] = self.data_store[0]
            self.data_store[0] = self.solar_data[self.data_count]
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
        self.data_store[0] = self.house_data[0]
        self.data_count = 0

        # Server read function
        @self.app.route(slave_ids=[1], function_codes=[3, 4, 6, 16], addresses=list(range(0, 1)))
        def read_data_store(slave_id, function_code, address):
            self.data_count += 1
            self.data_store[1] = self.data_store[0]
            self.data_store[0] = self.house_data[self.data_count]
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



