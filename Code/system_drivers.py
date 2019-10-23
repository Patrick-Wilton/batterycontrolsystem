
import struct
import threading
import time
from _thread import get_ident

import numpy as np
import zmq
from sunspec.core.client import ClientDevice


class Event:
    def __init__(self):
        self.events = dict()

    def wait(self):
        identifier = get_ident()
        if identifier not in self.events:
            self.events[identifier] = [threading.Event(), time.time()]
        return self.events[identifier][0].wait()

    def set(self):
        now = time.time()
        remove = None
        for identifier, event in self.events.items():
            if not event[0].isSet():
                event[0].set()
                event[1] = now
            else:
                if now - event[1] > 3:
                    remove = identifier
        if remove:
            del self.events[remove]

    def clear(self):
        self.events[get_ident()][0].clear()


class SunSpecDriver:
    def __init__(self, config_settings):

        # Reads settings configuration file
        self.settings = config_settings

        # Connecting SunSpec Clients
        print(self.settings.server["battery"]["ipaddr"])
        self.battery_client = ClientDevice(device_type=self.settings.server["battery"]["device_type"],
                                           slave_id=self.settings.server["battery"]["slave_id"],
                                           ipaddr=self.settings.server["battery"]["ipaddr"],
                                           ipport=self.settings.server["battery"]["ipport"])
        self.solar_client = ClientDevice(device_type=self.settings.server["solar"]["device_type"],
                                         slave_id=self.settings.server["solar"]["slave_id"],
                                         ipaddr=self.settings.server["solar"]["ipaddr"],
                                         ipport=self.settings.server["solar"]["ipport"])
        self.house_client = ClientDevice(device_type=self.settings.server["house"]["device_type"],
                                         slave_id=self.settings.server["house"]["slave_id"],
                                         ipaddr=self.settings.server["house"]["ipaddr"],
                                         ipport=self.settings.server["house"]["ipport"])

        # Event Classes
        self.bat_event = Event()
        self.solar_event = Event()
        self.house_event = Event()

        # Publisher Time Variables
        self.bat_time = self.settings.ZeroMQ["battery_pub_time"]
        self.solar_time = self.settings.ZeroMQ["solar_pub_time"]
        self.house_time = self.settings.ZeroMQ["house_pub_time"]

        # Connection Variables
        self.battery_connect = 0
        self.solar_connect = 0
        self.house_connect = 0
        self.initial_time = None

        # Parameters in case of simultaneous battery read and writes
        self.bat_read = 0
        self.bat_write = 0

        # ZeroMQ Settings
        self.bat_port = str(self.settings.ZeroMQ["battery_SOC_port"])
        self.solar_port = str(self.settings.ZeroMQ["solar_port"])
        self.house_port = str(self.settings.ZeroMQ["house_port"])
        self.sub_port = str(self.settings.ZeroMQ["battery_power_port"])
        self.batterySOC_topic = self.settings.ZeroMQ["battery_SOC_topic"]
        self.solar_topic = self.settings.ZeroMQ["solar_topic"]
        self.house_topic = self.settings.ZeroMQ["solar_topic"]
        self.batteryW_topic = str(self.settings.ZeroMQ["battery_power_topic"])

        # Defines Sockets and Threads
        self.bat_socket = None
        self.solar_socket = None
        self.house_socket = None
        self.sub_socket = None

        self.bat_pub_thread = None
        self.solar_thread = None
        self.house_thread = None
        self.bat_sub_thread = None

        # Starts Drivers
        self.start_drivers()

    def start_drivers(self):
        context = zmq.Context()
        self.bat_socket = context.socket(zmq.PUB)
        self.bat_socket.bind("tcp://*:%s" % self.bat_port)
        self.solar_socket = context.socket(zmq.PUB)
        self.solar_socket.bind("tcp://*:%s" % self.solar_port)
        self.house_socket = context.socket(zmq.PUB)
        self.house_socket.bind("tcp://*:%s" % self.house_port)
        self.sub_socket = context.socket(zmq.SUB)

        # Starts Battery SOC Driver Thread
        print('starting battery SOC driver')
        self.bat_pub_thread = threading.Thread(target=self.battery_publisher)
        self.bat_pub_thread.start()

        # Starts Solar Driver Thread
        print('starting solar driver')
        self.solar_thread = threading.Thread(target=self.solar_publisher)
        self.solar_thread.start()

        # Starts House Driver Thread
        print('starting house driver')
        self.house_thread = threading.Thread(target=self.house_publisher)
        self.house_thread.start()

        # Starts Battery Subscriber Thread
        print('starting battery subscriber')
        self.bat_sub_thread = threading.Thread(target=self.battery_subscriber)
        self.bat_sub_thread.start()

    def battery_publisher(self):
        battery_soc_decode = struct.pack(">h", int(self.settings.battery["initial_SOC"]))
        while True:
            # Makes sure initial connection is established
            if self.battery_connect == 0:
                self.bat_socket.send_string("%d %s" % (self.batterySOC_topic, 'bat_connect'))
            else:
                # SunSpec Reading and Decoding
                self.bat_read = 1
                if self.bat_write == 0:
                    battery_soc_decode = self.battery_client.read(self.settings.server["battery"]["SOCaddr"], 1)
                else:
                    print('skip battery READ')
                self.bat_read = 0

                soc_value = np.int16(int.from_bytes(battery_soc_decode, byteorder='big'))

                # ZeroMQ Publishing
                self.bat_socket.send_string("%d %d" % (self.batterySOC_topic, soc_value))

                # Publishing Method
                if self.settings.ZeroMQ["use_event_pub"]:
                    self.bat_event.wait()
                    self.bat_event.clear()
                else:
                    time.sleep(self.bat_time - ((time.time() - self.initial_time) % self.bat_time))

    def solar_publisher(self):
        while True:
            # Makes sure initial connection is established
            if self.solar_connect == 0:
                self.solar_socket.send_string("%d %s" % (self.solar_topic, 'solar_connect'))
            else:
                # SunSpec Reading and Decoding
                solar_decode = self.solar_client.read(self.settings.server["solar"]["poweraddr"], 1)
                solar_value = np.int16(int.from_bytes(solar_decode, byteorder='big'))

                self.solar_socket.send_string("%d %d" % (self.solar_topic, solar_value))

                # Publishing Method
                if self.settings.ZeroMQ["use_event_pub"]:
                    self.solar_event.wait()
                    self.solar_event.clear()
                else:
                    time.sleep(self.solar_time - ((time.time() - self.initial_time) % self.solar_time))

    def house_publisher(self):
        while True:
            # Makes sure initial connection is established
            if self.house_connect == 0:
                self.house_socket.send_string("%d %s" % (self.house_topic, 'house_connect'))
            else:
                # SunSpec Reading and Decoding
                house_decode = self.house_client.read(self.settings.server["house"]["poweraddr"], 1)
                house_value = np.int16(int.from_bytes(house_decode, byteorder='big'))

                self.house_socket.send_string("%d %d" % (self.house_topic, house_value))

                # Publishing Method
                if self.settings.ZeroMQ["use_event_pub"]:
                    self.house_event.wait()
                    self.house_event.clear()
                else:
                    time.sleep(self.house_time - ((time.time() - self.initial_time) % self.house_time))

    def battery_subscriber(self):
        # Connects Subscriber to socket and topic
        self.sub_socket.connect("tcp://localhost:%s" % self.sub_port)
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, self.batteryW_topic)

        while True:
            # ZeroMQ Subscribing
            power_string = self.sub_socket.recv()
            power_topic, bat_power = power_string.split()

            # Makes sure initial connection is established
            if bat_power == b'connected':
                self.initial_time = time.time()
                self.battery_connect = 1
                self.solar_connect = 1
                self.house_connect = 1
            else:
                # Writes new Power Value to Battery Server
                bat_power = int(bat_power)
                self.bat_write = 1
                if self.bat_read == 0:
                    self.battery_client.write(self.settings.server["battery"]["poweraddr"], struct.pack(">h", bat_power))
                else:
                    print('skip battery WRITE')

                # Sets to read new values from servers
                self.bat_event.set()
                self.bat_write = 0
                self.solar_event.set()
                self.house_event.set()

