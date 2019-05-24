

while True:
    '''

    Old while loop from client testing

    '''
    print('\n\nTest Iteration = ' + str(it_num))

    # Reads User Input
    TestInput = input('\nTest Options \n1 = Read Registers\n2 = Calculate New SOC\n'
                      '3 = Read SunSpec Model\nInput Number: ')

    # Reads SOC from Server
    if TestInput == '1':
        # Connects To Server
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('localhost', 8080))

        read = tcp.read_holding_registers(slave_id=1, starting_address=19, quantity=1)
        response = tcp.send_message(read, sock)
        print('\nSOC is Currently at: ' + str(response[0]) + ' Percent')

        # Closes Server
        sock.close()

    # Predicts New SOC and Adds to Data Store
    elif TestInput == '2':
        charge_str = input('\nInput the Charge (A)(+-): ')
        time_str = input('\nInput the Time the Charge is Applied (Hours): ')
        charge = float(charge_str)
        time = float(time_str)

        new_soc = battery.predict_soc(charge, time)

    # Reads from Server using SunSpec
    elif TestInput == '3':
        A = sun_client.read(0, 1)
        sun_spec_soc = int.from_bytes(A, byteorder='little')
        print(np.int16(sun_spec_soc))
        print('\nSOC is Currently at: ' + str(sun_spec_soc) + ' Percent')

    else:
        print('\nMust be a number between 1-3')
    it_num += 1