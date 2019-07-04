
import zmq
import time
import threading


class Subscriber:
    def __init__(self):

        self.battery_read = 0
        self.solar_read = 0
        self.house_read = 0

        # ZeroMQ Subscribing
        bat_port = "8090"
        solar_port = "8091"
        house_port = "8092"
        battery_topic = "0"
        solar_topic = "0"
        house_topic = "0"
        sub_context = zmq.Context()

        self.bat_socket = sub_context.socket(zmq.SUB)
        self.bat_socket.connect("tcp://localhost:%s" % bat_port)
        self.bat_socket.setsockopt_string(zmq.SUBSCRIBE, battery_topic)

        self.solar_socket = sub_context.socket(zmq.SUB)
        self.solar_socket.connect("tcp://localhost:%s" % solar_port)
        self.solar_socket.setsockopt_string(zmq.SUBSCRIBE, solar_topic)

        self.house_socket = sub_context.socket(zmq.SUB)
        self.house_socket.connect("tcp://localhost:%s" % house_port)
        self.house_socket.setsockopt_string(zmq.SUBSCRIBE, house_topic)

        # Starts Battery Sub Thread
        print('starting battery SOC subscriber')
        self.bat_thread = threading.Thread(target=self.battery_subscriber)
        self.bat_thread.start()

        # Starts Solar Sub Thread
        print('starting solar subscriber')
        self.solar_thread = threading.Thread(target=self.solar_subscriber)
        self.solar_thread.start()

        # Starts House Sub Thread
        print('starting house subscriber')
        self.house_thread = threading.Thread(target=self.house_subscriber)
        self.house_thread.start()

    def battery_subscriber(self):
        while True:
            bat_string = self.bat_socket.recv()
            self.b_topic, self.bat_SOC = bat_string.split()
            self.bat_SOC = int(self.bat_SOC)
            print('BATTERY')
            print(self.bat_SOC)
            self.battery_read = 1

    def solar_subscriber(self):
        while True:
            solar_string = self.solar_socket.recv()
            self.s_topic, self.solar_power = solar_string.split()
            self.solar_power = int(self.solar_power)
            print('SOLAR')
            print(self.solar_power)
            self.solar_read = 1

    def house_subscriber(self):
        while True:
            house_string = self.house_socket.recv()
            self.h_topic, self.house_power = house_string.split()
            self.house_power = int(self.house_power)
            print('HOUSE')
            print(self.house_power)
            self.house_read = 1


if __name__ == '__main__':

    # ZeroMQ Publishing
    pub_port = "8093"
    pub_topic = 0
    pub_context = zmq.Context()
    pub_socket = pub_context.socket(zmq.PUB)
    pub_socket.bind("tcp://*:%s" % pub_port)

    # Sets Initial Values
    prev_power = 0
    time_interval = 5  # minutes
    time_interval = 1/(60/time_interval)

    print('Starting Control System')
    sub = Subscriber()
    while True:
        if sub.battery_read == 1 & sub.solar_read == 1 & sub.house_read == 1:
            print('STEP')
            bat = sub.bat_SOC
            solar = sub.solar_power
            house = sub.house_power

            # Data Filtering

            # Control System
            grid = prev_power + solar + house
            bat_power = -grid
            prev_power = bat_power

            time.sleep(1)

            # Publishing
            pub_socket.send_string("%d %d" % (pub_topic, bat_power))

            # Reset Reads
            sub.battery_read = 0
            sub.solar_read = 0
            sub.house_read = 0




