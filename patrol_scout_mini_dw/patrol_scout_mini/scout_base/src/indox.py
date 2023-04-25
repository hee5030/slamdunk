from pymongo import MongoClient
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from scout_msgs.msg import ScoutStatus


client = MongoClient("mongodb+srv://tesless:123@cluster0.xyeyaz7.mongodb.net/test")

# print(client.list_database_names())

mydb = client['test']
mycol = mydb['customers']


def callback(msg):
      rospy.loginfo(rospy.get_caller_id() + "I heard %s")
      my_dict = [{"name":msg.header}]
      x = mycol.insert_many(my_dict)
      print(x.inserted_ids)

def main():
    image_topic = "ScoutStatus"
    
    rospy.init_node('ScoutStatus')
    
    rospy.Subscriber(image_topic, ScoutStatus, callback)
    rospy.spin()
    
if __name__ == '__main__':
    main()



