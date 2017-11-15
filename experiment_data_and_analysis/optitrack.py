import optirx as rx

file = open("optitrack_data.csv", 'w')

dsock = rx.mkdatasock()
version = (2, 9, 0, 0)  # NatNet version to use
while True:
	data = dsock.recv(rx.MAX_PACKETSIZE)
	packet = rx.unpack(data, version=version)
	if type(packet) is rx.SenderData:
	    version = packet.natnet_version
	frame = (packet.rigid_bodies[2].markers[0] + packet.rigid_bodies[2].markers[1] + packet.rigid_bodies[2].markers[2])
	file.write( str(frame)[1:-1] + "\n")




file.close()