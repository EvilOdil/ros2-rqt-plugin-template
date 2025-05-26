from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget, QFileDialog
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import os
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        self.setObjectName('MyPlugin')

        self.node = context.node  # Use shared RQT node
        self._widget = MyWidget(self.node)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + f' ({context.serial_number()})')

        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass  # No custom node to shut down


class MyWidget(QWidget):
    def __init__(self, node):
        super(MyWidget, self).__init__()
        self.node = node

        # Load UI
        ui_file = os.path.join(
            get_package_share_directory('rqt_mypkg'),
            'resource',
            'rqt_mypkg.ui'
        )
        loadUi(ui_file, self)
        self.setObjectName('MyWidget')

        # Access UI elements
        self.button = self.findChild(QWidget, 'pushButton_up')
        self.checkbox = self.findChild(QWidget, 'lightCheckBox')
        self.current_top_label = self.findChild(QWidget, 'label_5')
        self.voltage_top_label = self.findChild(QWidget, 'label_3')
        self.lights_top_label = self.findChild(QWidget, 'lights_top')


        # Connect UI actions
     
        if self.checkbox:
            self.checkbox.stateChanged.connect(self.on_checkbox_toggle)
   

        # ROS subscriptions
        self._ros_subs = []
        self._ros_subs.append(
            self.node.create_subscription(
                String,
                '/chatter',
                self.chatter_callback,
                10
            )
        )
        self._ros_subs.append(
            self.node.create_subscription(
                Twist,
                '/turtle1/cmd_vel',
                self.cmd_vel_callback,
                10
            )
        )
        self._ros_subs.append(
            self.node.create_subscription(
                String,  # Replace with actual message type if available
                '/turtle1/rotate_absolute',
                self.rotate_status_callback,
                10
            )
        )


    def on_checkbox_toggle(self, state):
        if state == Qt.Checked:
            self.lights_top_label.setText("Lights on")
        else:
            self.lights_top_label.setText("Lights off")



    def chatter_callback(self, msg):
        self.label_7.setText(f"Received: {msg.data}")

    def cmd_vel_callback(self, msg):
        # Update current_top_label with linear.x (as current)
        if self.current_top_label:
            self.current_top_label.setText(f"{msg.linear.x:.2f}")
        # Update voltage_top_label with angular.z (as voltage)
    

    def rotate_status_callback(self, msg):
        # Display rotate status in voltage_top_label (or elsewhere as needed)
        if self.voltage_top_label:
            self.voltage_top_label.setText(f"R {msg.data}")

    def closeEvent(self, event):
        # Explicitly remove references to subscriptions to help cleanup
        self._ros_subs.clear()
        super().closeEvent(event)
