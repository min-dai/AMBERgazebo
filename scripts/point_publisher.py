#!/usr/bin/env python3
#
#   point_publisher.py
#
#   Publish an x/y/z point, as determined by three sliders
#
#   This generates a point message, as well as a visualization marker
#   array, which rviz can render.
#
#   Publish:    /point                          visualization_msgs/MarkerArray
#   Publish:    /visualization_marker_array     visualization_msgs/MarkerArray
#
import sys
import signal
import threading
import rospy
 
from PyQt5.QtCore    import (Qt, QTimer)
from PyQt5.QtWidgets import (QWidget, QSlider, QHBoxLayout, QVBoxLayout,
                             QLabel, QApplication)

from geometry_msgs.msg      import Point
from geometry_msgs.msg      import PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#
#  Point Publisher
#
#  This continually publishes the point and corresponding marker array.
#
class PointPublisher:
    def __init__(self, p0):
        # Prepare the publishers (latching for new subscribers).
        self.pub_mark  = rospy.Publisher("/visualization_marker_array",
                                         MarkerArray,
                                         queue_size=1, latch=True)
        self.pub_point = rospy.Publisher("/point",
                                         PointStamped,
                                         queue_size=1, latch=True)

        # Create the point.
        self.p = Point()
        self.p.x = p0[0]
        self.p.y = p0[1]
        self.p.z = p0[2]

        # Create the sphere marker.
        self.marker = Marker()
        self.marker.header.frame_id    = "world"
        self.marker.header.stamp       = rospy.Time.now()
        self.marker.action             = Marker.ADD
        self.marker.ns                 = "point"
        self.marker.id                 = 1
        self.marker.type               = Marker.SPHERE
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position      = self.p
        self.marker.scale.x            = 0.2
        self.marker.scale.y            = 0.2
        self.marker.scale.z            = 0.2
        self.marker.color.r            = 1.0
        self.marker.color.g            = 1.0
        self.marker.color.b            = 1.0
        self.marker.color.a            = 0.5     # Make transparent!

        # Create the marker array message.
        self.mark = MarkerArray()
        self.mark.markers.append(self.marker)

        # Create the point message.
        self.point = PointStamped()
        self.point.header.frame_id = "world"
        self.point.header.stamp    = rospy.Time.now()
        self.point.point           = self.p

    def update(self, p):
        self.p.x = p[0]
        self.p.y = p[1]
        self.p.z = p[2]

    def publish(self):
        # Publish.
        now = rospy.Time.now()
        self.marker.header.stamp = now
        self.point.header.stamp  = now
        self.pub_mark.publish(self.mark)
        self.pub_point.publish(self.point)

    def loop(self):
        # Prepare a servo loop at 10Hz.
        servo = rospy.Rate(10)
        rospy.loginfo("Point-Publisher publication thread running at 10Hz...")

        # Loop: Publish and sleep
        while not rospy.is_shutdown():
            self.publish()
            servo.sleep()

        # Report the cleanup.
        rospy.loginfo("Point-Publisher publication thread ending...")


#
#  GUI Slider Class
#
class SingleVariable(QWidget):
    def __init__(self, name, val, minval, maxval, callback):
        super().__init__()
        self.value    = val
        self.offset   = (maxval + minval) / 2.0
        self.slope    = (maxval - minval) / 200.0
        self.callback = callback
        self.initUI(name)

    def initUI(self, name):
        # Top-Left: Name
        label = QLabel(name)
        label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        label.setMinimumWidth(40)

        # Top-Right: Number
        self.number = QLabel("%6.3f" % self.value)
        self.number.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.number.setMinimumWidth(100)
        self.number.setStyleSheet("border: 1px solid black;")

        # Bottom: Slider
        slider = QSlider(Qt.Horizontal)
        slider.setRange(-100, 100)
        slider.setFocusPolicy(Qt.NoFocus)
        slider.setPageStep(5)
        slider.valueChanged.connect(self.valueHandler)
        slider.setValue(int((self.value - self.offset)/self.slope))

        # Create the Layout
        hbox = QHBoxLayout()
        hbox.addWidget(label)
        hbox.addSpacing(10)
        hbox.addWidget(self.number)

        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        vbox.addWidget(slider)

        self.setLayout(vbox)

    def valueHandler(self, value):
        self.value = self.offset + self.slope * float(value)
        self.number.setText("%6.3f" % self.value)
        self.callback(self.value)

    
class XYZGUI(QWidget):
    def __init__(self, p0, callback):
        super().__init__()
        self.value    = p0
        self.callback = callback
        self.initUI(p0)

    def initUI(self, p):
        # Create the XYZ variables
        vbox = QVBoxLayout()
        vbox.addWidget(SingleVariable('X', p[0], -1.0, 1.0, self.xHandler))
        vbox.addWidget(SingleVariable('Y', p[1], -1.0, 1.0, self.yHandler))
        vbox.addWidget(SingleVariable('Z', p[2],  0.0, 2.0, self.zHandler))

        self.setLayout(vbox)
        self.setWindowTitle('XYZ Position')
        self.show()

    def xHandler(self, value):
        self.value[0] = value
        self.callback(self.value)

    def yHandler(self, value):
        self.value[1] = value
        self.callback(self.value)

    def zHandler(self, value):
        self.value[2] = value
        self.callback(self.value)

    def kill(self, signum, frame):
        self.close()


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare the node.
    rospy.init_node('point_publish')

    # Pick an initial value.
    p0 = [0.5, 0.5, 0.5]

    # Prepare the point publisher.
    ppub = PointPublisher(p0)

    # Prepare Qt.
    app = QApplication(sys.argv)
    app.setApplicationDisplayName("Point Publisher")

    # Include a Qt Timer, setup up so every 500ms the python
    # interpreter runs (doing nothing).  This enables the ctrl-c
    # handler to be processed and close the window.
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    # Then setup the GUI window.  And declare the ctrl-c handler to
    # close that window.
    gui = XYZGUI(p0, ppub.update)
    signal.signal(signal.SIGINT, gui.kill)


    # Start the publisher in a separate thread.
    ppubthread = threading.Thread(target=ppub.loop)
    ppubthread.start()

    # Start the GUI window.
    rospy.loginfo("Point-Publisher GUI starting...")
    status = app.exec_()
    rospy.loginfo("Point-Publisher GUI ended.")

    # Having re-mapped the sigint handler, we have to shutdown ROS
    # manually (and thus the publisher thread).
    rospy.signal_shutdown("")
    ppubthread.join()
    
    # And exit.
    sys.exit(status)
