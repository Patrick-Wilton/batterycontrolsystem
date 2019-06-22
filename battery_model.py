"""

For a more complex battery model

"""

import zmq
import time

# ZeroMQ Publishing
port = "8090"
topic = 0
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:%s" % port)

while True:
    socket.send_string("%d %d" % (topic, 1000))

