#!/usr/bin/env python
# license removed for brevity
import rospy
##from std_msgs.msg import String
from std_msgs.msg import Float32
import socket
from time import gmtime, strftime
import ConfigParser
import time



class socket_connection ():

    def __init__ (self, publisher_object, rate):

        try:

            config = ConfigParser.ConfigParser()
            config.read("config.ini")

            self.host_ip = config.get('Information', 'ip_address_ur5')
            self.port = int (config.get ('Information', 'port'))
            self.output_file_location = config.get ('Information', 'output_file_location') 
            
            self.write_object = True

            if self.output_file_location is "":
                self.output_file_location = ""
            elif self.output_file_location is "None":
                self.write_object = False
            else:
                self.output_file_location = self.output_file_location + "/"

                
        except:

            print ("Warning: Config.ini file problem, please check Config.ini")

        self.socket_object = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.publisher_object = publisher_object
        self.publisher_rate = rate
        self.is_published = False


    def connect (self):

        try:

            print("Warning: Connecting to Ur5 ip adress: " + self.host_ip)
            self.socket_object.connect(( self.host_ip, self.port ))

            if self.write_object is True:
                print("Warning: File Write Location: " + self.output_file_location)
                f = open (self.output_file_location + "DataStream.csv", "w")
                       
            try:
                print("Writing in DataStream.csv, Press ctrl + c to stop")
                while (1):

                    deneme = self.socket_object.recv(1024)
                    denemeF = float(deneme.split(",")[2])

##                    force_torque_values = (self.socket_object.recv(1024).replace("(","*")).replace(")","*")
                    
##                    value_list = force_torque_values.split("*")
                    
##                    rospy.loginfo(value_list [1])
                    rospy.loginfo(denemeF)



##                    self.publisher_object.publish(value_list[1])
                    self.publisher_object.publish(denemeF)
                    self.publisher_rate.sleep()

##                    if self.write_object is True:       
##                        f.write(strftime("%a, %d %b %Y %H:%M:%S +0000", gmtime()) + ": " +force_torque_values)
                                              
            except KeyboardInterrupt:
                        
                f.close
                self.socket_object.close

                return False

        except Exception as e:

            print("Error: No Connection!! Please check your ethernet cable :)" + str(e))

            return False



def main():
    

##    pub = rospy.Publisher('ft300_force_torque', String, queue_size=10)
    
    pub = rospy.Publisher('ft300_force_torque', Float32, queue_size=10)    
    rospy.init_node('torque_force_sensor_data', anonymous=True)
        
    rate = rospy.Rate(10) 

    socket_connection_obj = socket_connection(pub, rate)    
    socket_connection_obj.connect()
          

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass                         

    
                        