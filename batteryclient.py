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
import socket
from umodbus import conf
from umodbus.client import tcp
from batteryserver import Battery


if __name__ == '__main__':

    # Initialises Test Number and Battery Server
    it_num = 1
    battery = Battery()

    # Enable values to be signed (default is False).
    conf.SIGNED_VALUES = True

    # Connects To Server
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('localhost', 8080))

    try:
        while True:
            print('\n\nTest Iteration = ' + str(it_num))

            # Reads User Input
            TestInput = input('\nTest Options \n1 = Read Registers\n2 = Calculate New SOC\n'
                              '3 = Read SunSpec Model\nInput Number: ')

            # Reads SOC from Server
            if TestInput == '1':

                read = tcp.read_holding_registers(slave_id=1, starting_address=19, quantity=1)
                response = tcp.send_message(read, sock)
                print(response)

            # Predicts New SOC and Adds to Data Store
            elif TestInput == '2':
                charge_str = input('\nInput the Charge (A)(+-): ')
                time_str = input('\nInput the Time the Charge is Applied (Hours): ')
                charge = float(charge_str)
                time = float(time_str)

                new_soc = battery.predict_soc(charge, time)

            # Reads from Server using SunSpec
            elif TestInput == '3':
                print('To be implemented')

            else:
                raise ValueError('Must be a number between 1-3')

            it_num += 1

    finally:
        sock.close()
