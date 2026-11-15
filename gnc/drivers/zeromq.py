# We will get the zeromq messages here


import zmq


addr = "tcp://localhost:1"  # this is just a placeholder -> look into architecture for tcp vs udp for latency
# have to put / look into latency difference between fpga and this since we have to wait for fpga to dev

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect(addr)


reply = socket.rec()  # stores the recieve
