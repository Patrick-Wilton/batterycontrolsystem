
import zmq
import time

if __name__ == '__main__':

    # ZeroMQ Subscribing
    sub_port = "8090"
    battery_topic = "0"
    solar_topic = "1"
    house_topic = "2"
    sub_context = zmq.Context()

    bat_socket = sub_context.socket(zmq.SUB)
    bat_socket.connect("tcp://localhost:%s" % sub_port)
    bat_socket.connect("tcp://localhost:%s" % sub_port)
    bat_socket.setsockopt_string(zmq.SUBSCRIBE, battery_topic)

    solar_socket = sub_context.socket(zmq.SUB)
    solar_socket.connect("tcp://localhost:%s" % sub_port)
    solar_socket.connect("tcp://localhost:%s" % sub_port)
    solar_socket.setsockopt_string(zmq.SUBSCRIBE, solar_topic)

    house_socket = sub_context.socket(zmq.SUB)
    house_socket.connect("tcp://localhost:%s" % sub_port)
    house_socket.connect("tcp://localhost:%s" % sub_port)
    house_socket.setsockopt_string(zmq.SUBSCRIBE, house_topic)

    # ZeroMQ Publishing
    pub_port = "8091"
    pub_topic = 0
    pub_context = zmq.Context()
    pub_socket = pub_context.socket(zmq.PUB)
    pub_socket.bind("tcp://*:%s" % pub_port)

    # Sets Initial Values
    prev_power = 0
    time_interval = 5  # minutes
    time_interval = 1/(60/time_interval)

    print('Starting Control System')
    while True:
        # Subscribing
        bat_string = bat_socket.recv()
        solar_string = solar_socket.recv()
        house_string = house_socket.recv()

        # Obtains Battery SOC
        b_topic, bat_SOC = bat_string.split()
        bat_SOC = int(bat_SOC)
        print('BATTERY')
        print(bat_SOC)

        # Obtains Solar Power
        s_topic, solar_power = solar_string.split()
        solar_power = int(solar_power)
        print('SOLAR')
        print(solar_power)

        # Obtains House Load
        h_topic, house_power = house_string.split()
        house_power = int(house_power)
        print('HOUSE')
        print(house_power)

        # Data Filtering

        # Control System
        grid = prev_power + solar_power + house_power
        bat_power = -grid
        prev_power = bat_power

        # Publishing
        pub_socket.send_string("%d %d" % (pub_topic, bat_power))
        time.sleep(1)



