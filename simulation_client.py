import csv
import threading
import time
from _thread import get_ident

import numpy as np
import zmq
from sunspec.core.client import ClientDevice

from simulation_servers import Battery, Solar, House


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

        # Setting up Data
        battery_data = []
        solar_data = []
        house_data = []
        with open('one_day_export.csv', mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                battery_data.append(int((float(row["abatteryp"]) * 1000)))
                solar_data.append(int(float(row["asolarp"]) * 1000))
                house_data.append(int(float(row["aloadp"]) * 1000))

        # Creating Servers
        self.battery = Battery(battery_data)
        self.solar = Solar(solar_data)
        self.house = House(house_data)

        # Connecting SunSpec Clients
        self.battery_client = ClientDevice(device_type='TCP', slave_id=1, ipaddr='localhost', ipport=8080)
        self.solar_client = ClientDevice(device_type='TCP', slave_id=1, ipaddr='localhost', ipport=8081)
        self.house_client = ClientDevice(device_type='TCP', slave_id=1, ipaddr='localhost', ipport=8082)

        # Waits for Servers
        time.sleep(0.5)

        # Event Class
        self.event = Event()

        # ZeroMQ Publishing
        self.pub_port = "8090"
        self.batterySOC_topic = 0
        self.solar_topic = 1
        self.house_topic = 2
        self.context = zmq.Context()
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind("tcp://*:%s" % self.pub_port)

        # Connection Variables
        self.battery_connect = 1
        self.solar_connect = 1
        self.house_connect = 1

        # ZeroMQ Subscribing
        self.sub_port = "8091"
        self.batteryW_topic = 1
        self.sub_socket = self.context.socket(zmq.SUB)

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
        while True:
            if self.battery_connect == 1:
                # SunSpec Reading and Decoding
                battery_soc_decode = self.battery_client.read(19, 1)
                soc_value = np.int16(int.from_bytes(battery_soc_decode, byteorder='big'))
                print('SOC')
                print(soc_value)
                self.battery_connect = 0
            else:
                self.pub_socket.send_string("%d %d" % (self.batterySOC_topic, soc_value))

    def solar_publisher(self):
        while True:
            if self.solar_connect == 1:
                # SunSpec Reading and Decoding
                solar_decode = self.solar_client.read(0, 1)
                solar_value = np.int16(int.from_bytes(solar_decode, byteorder='big'))
                print('SOLAR')
                print(solar_value)
                self.solar_connect = 0
            else:
                self.pub_socket.send_string("%d %d" % (self.solar_topic, solar_value))

    def house_publisher(self):
        while True:
                # SunSpec Reading and Decoding
                house_decode = self.house_client.read(0, 1)
                house_value = np.int16(int.from_bytes(house_decode, byteorder='big'))

                print('HOUSE')
                print(house_value)

                self.pub_socket.send_string("%d %d" % (self.house_topic, house_value))

                self.event.wait()
                self.event.clear()

    def battery_subscriber(self):
        self.sub_socket.connect("tcp://localhost:%s" % self.sub_port)
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "0")
        while True:
            # Power Value Subscribing
            power_string = self.sub_socket.recv()
            power_topic, bat_power = power_string.split()
            bat_power = int(bat_power)
            print('BAT_POWER')
            print(bat_power)

            # Write to Battery Server
            self.battery.predict_soc(bat_power)

            # Sets to read new values from servers
            self.event.set()
            self.battery_connect = 1
            self.solar_connect = 1
            #self.house_connect = 1


if __name__ == '__main__':

    # Starting SunSpec Driver
    sun_spec_driver = SunSpecDriver()

