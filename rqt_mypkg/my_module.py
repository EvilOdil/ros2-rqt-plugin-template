#!/usr/bin/env python3

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget, QFileDialog
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import os
from ament_index_python.packages import get_package_share_directory


class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        self.setObjectName('MyPlugin')

        self._widget = MyWidget()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + f' ({context.serial_number()})')

        context.add_widget(self._widget)


class MyWidget(QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()

        # Load UI file
        ui_file = os.path.join(
        get_package_share_directory('rqt_mypkg'),
        'resource',
        'rqt_mypkg.ui'
       )
        ui_file = os.path.abspath(ui_file)
        loadUi(ui_file, self)
        self.setObjectName('MyWidget')

        # Access widgets by objectName set in Qt Designer
        self.button = self.findChild(QWidget, 'pushButton')
        self.label = self.findChild(QWidget, 'statusLabel')
        self.image_label = self.findChild(QWidget, 'imageLabel')
        self.checkbox = self.findChild(QWidget, 'enableCheck')
        self.load_image_button = self.findChild(QWidget, 'loadImageButton')

        # Connect signals
        if self.button:
            self.button.clicked.connect(self.on_button_click)

        if self.checkbox:
            self.checkbox.stateChanged.connect(self.on_checkbox_toggle)

        if self.load_image_button:
            self.load_image_button.clicked.connect(self.select_image)

    def on_button_click(self):
        self.label.setText("Button Clicked!")

    def on_checkbox_toggle(self, state):
        if state == Qt.Checked:
            self.label.setText("Visual mode enabled")
        else:
            self.label.setText("Visual mode disabled")

    def select_image(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select Image File", "", "Images (*.png *.jpg *.jpeg *.bmp *.gif)"
        )
        if file_path:
            pixmap = QPixmap(file_path)
            if not pixmap.isNull():
                self.image_label.setPixmap(pixmap.scaled(
                    self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
                self.label.setText("Image loaded successfully!")
            else:
                self.label.setText("Failed to load image.")
