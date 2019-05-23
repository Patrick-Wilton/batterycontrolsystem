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

# Import Necessary Libraries for TCP server
import logging
import threading
from collections import defaultdict
from socketserver import TCPServer

from umodbus import conf
from umodbus.server.tcp import RequestHandler, get_server
from umodbus.utils import log_to_stream


class Server:
    def __init__(self):
        # Initialises Data Store
        self.data_store = defaultdict(int)

        # Enable values to be signed
        conf.SIGNED_VALUES = False

        # Add Stream Handler to Logger
        log_to_stream(level=logging.DEBUG)

        # Creates TCP Server
        TCPServer.allow_reuse_address = True
        self.app = get_server(TCPServer, ('127.0.0.1', 8081), RequestHandler)

        # Server read function
        @self.app.route(slave_ids=[1], function_codes=[3, 4, 6, 16], addresses=list(range(0, 34)))
        def read_data_store(slave_id, function_code, address):
            return self.data_store[address]

        # Server write function
        @self.app.route(slave_ids=[1], function_codes=[6, 16], addresses=list(range(0, 34)))
        def write_data_store(slave_id, function_code, address, value):
            self.data_store[address] = value

        # Starting server in background thread
        Server.thread = threading.Thread(target=self._thread)
        Server.thread.start()

    def _thread(self):
        try:
            print('starting server')
            self.app.serve_forever()
        finally:
            self.app.shutdown()
            self.app.server_close()
            Server.thread = None


class Solar:
    def __init__(self):
        # Server Instance
        self.server = Server()

        # Sets Initial Values
        self.power = 50
        self.dt = 0.1
        self.bat_cap = 5
        self.server.data_store[0] = 803
        self.server.data_store[1] = 16
        self.server.data_store[19] = 50
        print('battery initialised')

    def set_value(self, new_soc):
        self.SOC = new_soc
        self.server.data_store[19] = new_soc

    def return_value(self):
        return self.SOC

    def predict_soc(self, charge, time):
        old_soc = self.return_value()

        for dt in range(round(time/self.dt)):
            new_soc = old_soc + (charge/self.bat_cap)*self.dt
            old_soc = new_soc
        new_soc = round(new_soc)

        if new_soc > 100:
            raise ValueError('State of Charge has exceeded 100%')

        elif new_soc < 0:
            raise ValueError('State of Charge has fallen below 0%')

        self.set_value(new_soc)
        return new_soc

