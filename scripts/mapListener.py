#!/usr/bin/python
import sys
import os
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
import array

class MapListener(object):

    def __init__(self):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.sub = rospy.Subscriber("map", OccupancyGrid, self.mapConvert)

        # Initialize message variables.
        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.sub = rospy.Subscriber("map", OccupancyGrid, self.mapConvert)

    def mapConvert(self, msg):
        """Handle subscriber data."""
        header = msg.header

        # ros map saver
        mapdatafile = "testmap.pgm"
        rospy.loginfo("Writing map occupancy data to %s", mapdatafile)

        """
        try:
            with open(mapdatafile, 'wb') as out:
                #out.write("P5\n# CREATOR: map_saver.cpp %s m/pix\n%s %s\n255\n" % (msg.info.resolution, msg.info.width, msg.info.height))
                out.write(bytearray('P5' + ' ' + str(msg.info.width) + ' ' + str(msg.info.height) + ' ' + str(255) +  '\n', "utf-8"))
                for y in range(np.uint8(0), msg.info.height):
                    for x in range(np.uint8(0), msg.info.width):
                        i = np.uint8(x + (msg.info.height - y - 1) * msg.info.width)
                        if(msg.data[i] >= 0 and msg.data[i] <= 0):
                            #occ [0,0.1)
                            out.write(bytearray(str(254), "utf-8"))
                        elif(msg.data[i] <= 100 and msg.data[i] >= 100):
                            #occ (0.65,1]
                            out.write(bytearray(str(000), "utf-8"))
                        else:
                            #occ [0.1,0.65]
                            out.write(bytearray(str(205), "utf-8"))
                out.close()
        except IOError:
            rospy.loginfo("Couldn't save map file to %s", mapdatafile)
        """

        # declare 1-d array of unsigned char and assign it with values
        buff=array.array('B')

        for i in range(0, msg.info.width*msg.info.height):
            if(msg.data[i] >= 0 and msg.data[i] <= 0):
                buff.append(254)
            elif(msg.data[i] <= 100 and msg.data[i] >= 100):
                buff.append(000)
            else:
                buff.append(205)

        # open file for writing
        try:
           out = open(mapdatafile, 'wb')
        except IOError:
            rospy.loginfo("Couldn't save map file to %s", mapdatafile)

        # write the header to the file
        pgmHeader = 'P5' + '\n' + str(msg.info.width) + '  ' + str(msg.info.height) + '  ' + str(255) + '\n'
        out.write(bytearray(pgmHeader, "utf-8"))

        # write the data to the file
        buff.tofile(out)

        # close the file
        out.close()

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.sub.unregister()


if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = "MapListenerNode"
    rospy.init_node(node_name, anonymous=True)

    mapListener = MapListener()

    # Go to the main loop
    try:
        mapListener.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        mapListener.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)


        print("Node stopped")
