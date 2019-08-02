
import struct
import threading
import time
from _thread import get_ident

import numpy as np
import zmq
from sunspec.core.client import ClientDevice


class Event:
    def __init__(self):
        self.events = {}

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
    def __init__(self):

        # Driver Settings
        self.bat_int = 0.1
        self.solar_int = 0.1
        self.house_int = 0.1

        self.bat_pub_time = False
        self.solar_pub_time = False
        self.house_pub_time = False
        self.all_pub_time = False

        # Connecting SunSpec Clients
        self.battery_client = ClientDevice(device_type='TCP', slave_id=1, ipaddr='localhost', ipport=8080)
        self.solar_client = ClientDevice(device_type='TCP', slave_id=1, ipaddr='localhost', ipport=8081)
        self.house_client = ClientDevice(device_type='TCP', slave_id=1, ipaddr='localhost', ipport=8082)

        # Event Classes
        self.bat_event = Event()
        self.solar_event = Event()
        self.house_event = Event()

        # Connection Variables
        self.battery_connect = 0
        self.solar_connect = 0
        self.house_connect = 0

        # Parameters in case of simultaneous battery read and writes
        self.bat_read = 0
        self.bat_write = 0

        # ZeroMQ Settings
        self.bat_port = "8090"
        self.solar_port = "8091"
        self.house_port = "8092"
        self.batterySOC_topic = 0
        self.solar_topic = 0
        self.house_topic = 0
        self.sub_port = "8093"
        self.batteryW_topic = "0"

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
        battery_soc_decode = b'\x00\x00'  # TODO set actual init value defined
        while True:
            # Makes sure initial connection is established
            if self.battery_connect == 0:
                self.bat_socket.send_string("%d %s" % (self.batterySOC_topic, 'bat_connect'))
            else:
                # SunSpec Reading and Decoding
                self.bat_read = 1
                if self.bat_write == 0:
                    battery_soc_decode = self.battery_client.read(19, 1)
                else:
                    print('skip battery READ')
                self.bat_read = 0

                soc_value = np.int16(int.from_bytes(battery_soc_decode, byteorder='big'))

                # ZeroMQ Publishing
                self.bat_socket.send_string("%d %d" % (self.batterySOC_topic, soc_value))

                if self.bat_pub_time or self.all_pub_time:
                    time.sleep(self.bat_int)
                else:
                    self.bat_event.wait()
                    self.bat_event.clear()

    def solar_publisher(self):
        while True:
            # Makes sure initial connection is established
            if self.solar_connect == 0:
                self.solar_socket.send_string("%d %s" % (self.solar_topic, 'solar_connect'))
            else:
                # SunSpec Reading and Decoding
                solar_decode = self.solar_client.read(0, 1)
                solar_value = np.int16(int.from_bytes(solar_decode, byteorder='big'))

                self.solar_socket.send_string("%d %d" % (self.solar_topic, solar_value))

                if self.solar_pub_time or self.all_pub_time:
                    time.sleep(self.solar_int)
                else:
                    self.solar_event.wait()
                    self.solar_event.clear()

    def house_publisher(self):
        while True:
            # Makes sure initial connection is established
            if self.house_connect == 0:
                self.house_socket.send_string("%d %s" % (self.house_topic, 'house_connect'))
            else:
                # SunSpec Reading and Decoding
                house_decode = self.house_client.read(0, 1)
                house_value = np.int16(int.from_bytes(house_decode, byteorder='big'))

                self.house_socket.send_string("%d %d" % (self.house_topic, house_value))

                if self.house_pub_time or self.all_pub_time:
                    time.sleep(self.house_int)
                else:
                    self.house_event.wait()
                    self.house_event.clear()

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
                self.battery_connect = 1
                self.solar_connect = 1
                self.house_connect = 1
            else:
                # Writes new Power Value to Battery Server
                bat_power = int(bat_power)
                self.bat_write = 1
                if self.bat_read == 0:
                    self.battery_client.write(3, struct.pack(">h", bat_power))
                else:
                    print('skip battery WRITE')

                # Sets to read new values from servers
                self.bat_event.set()
                self.bat_write = 0
                self.solar_event.set()
                self.house_event.set()

