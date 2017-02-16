#! /usr/bin/env python

import rospy
from std_msgs.msg import Empty
from ftplib import FTP
from io import BytesIO
# See http://bebop-autonomy.readthedocs.io/en/latest/piloting.html
# #take-on-board-snapshot
from my_flickr_api import upload
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from test_client import pose


def point(x, y, z):
    p = [x, y, z]
    r = PointStamped()
    r.header.frame_id = 'World'
    r.point = Point(*p)
    return r


class Photographer():

    def __init__(self):
        rospy.init_node('photographer')
        self.place = pose(*rospy.get_param("~place"))
        self.observe = point(*rospy.get_param("~observe"))
        self.snapshot_pub = rospy.Publisher("snapshot", Empty,
                                            queue_size=1)
        rospy.Subscriber('take_photo', Empty, self.take_photo, queue_size=1)
        rospy.Subscriber('go_photo', Empty, self.go_photo)
        self.target_pub = rospy.Publisher(
            'target', PoseStamped, queue_size=1)
        self.observe_pub = rospy.Publisher(
            'observe', PointStamped, queue_size=1)
        rospy.loginfo("Will observe %s", self.observe)
        rospy.spin()

    def go_photo(self, msg):
        self.target_pub.publish(self.place)
        self.observe_pub.publish(self.observe)

    def retrieve_photos(self):
        photos = []
        rospy.loginfo("try to connect to ftp server")
        ftp = FTP('192.168.42.1', timeout=30)
        ftp.login()
        rospy.loginfo('Connected')
        ftp.cwd('internal_000/Bebop_2/media')
        while True:
            file_names = [f for f in ftp.nlst() if 'jpg' in f]
            # rospy.loginfo("Files %s", file_names)
            if file_names:
                for name in file_names:
                    rospy.loginfo("Try to retrieve %s", name)
                    f = BytesIO()
                    cmd = 'RETR {f}'.format(f=name)
                    ftp.retrbinary(cmd, f.write)
                    f.seek(0)
                    photos.append((name, f))
                    ftp.delete(name)
                return photos
            rospy.sleep(1)

    def take_photo(self, msg):
        rospy.loginfo("take_photo")
        self.snapshot_pub.publish()
        images = self.retrieve_photos()
        rospy.loginfo("Took photos %s", len(images))
        for name, image in images:
            rospy.loginfo(
                "Try to upload %s to flickr", name)
            s = upload(
                photo_file=name, title="Test bebop", photo_file_data=image)
            rospy.loginfo("Uploaded %s", s)


if __name__ == '__main__':
    try:
        Photographer()
    except rospy.ROSInterruptException:
        print "Program interrupted"
