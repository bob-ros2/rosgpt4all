#!/usr/bin/env python3
#
# Copyright 2023 BobRos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import os
import sys
import select
import fcntl
import signal
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor

# ROS node class

class RosNode(Node):
    """Basic ROS Node"""
    def __init__(self):
        super().__init__('terminal')

        self.pub = self.create_publisher(String, 'gpt_in', 10)
        self.sub1 = None
        self.sub2 = None

        self.declare_parameter('title', 'GPT4ALL Terminal', ParameterDescriptor(
            description='Title of window.'))
        self.declare_parameter('opacity', 1.0, ParameterDescriptor(
            description='Window opacity.'))
        self.declare_parameter('frameless', False, ParameterDescriptor(
            description='Switch off window caption.'))
        self.declare_parameter('fontname', 'courier', ParameterDescriptor(
            description='Window fontname.'))
        self.declare_parameter('fontsize', 15, ParameterDescriptor(
            description='Window fontsize.'))
        self.declare_parameter('geometry', [0,0], ParameterDescriptor(
            description='Window geometry. [x, y, with, height]'))
        self.declare_parameter('display', -1, ParameterDescriptor(
            description='Display where to show window.'))
        self.declare_parameter('margin', [10,0,0,0], ParameterDescriptor(
            description='Window inner margin. [left, top, right, bottom]'))
        self.declare_parameter('input', True, ParameterDescriptor(
            description='Enables or disables the text input field.'))
        self.declare_parameter('stylesheet', 
            "background-color: black; color: #e9e9e9;", 
            ParameterDescriptor(
            description='Stylesheet qss of PlainText area.'))
        self.declare_parameter('stylesheet_window', 
            "background-color: black; color white;", 
            ParameterDescriptor(
            description='Stylesheet qss of Window area.'))
        self.declare_parameter('line_count', 0, 
            ParameterDescriptor(
            description="Maximum line count in the text area. 0 = unlimited\n"
            "If the number exceeds the lines are removed from the top."))

        self.title = self.get_parameter(
            'title').get_parameter_value().string_value
        self.opacity = self.get_parameter(
            'opacity').get_parameter_value().double_value
        self.frameless = self.get_parameter(
            'frameless').get_parameter_value().bool_value
        self.fontname = self.get_parameter(
            'fontname').get_parameter_value().string_value
        self.fontsize = self.get_parameter(
            'fontsize').get_parameter_value().integer_value
        self.geometry = self.get_parameter(
            'geometry').get_parameter_value().integer_array_value
        self.display = self.get_parameter(
            'display').get_parameter_value().integer_value
        self.stylesheet = self.get_parameter(
            'stylesheet').get_parameter_value().string_value
        self.stylesheet_window = self.get_parameter(
            'stylesheet_window').get_parameter_value().string_value
        self.margin = self.get_parameter(
            'margin').get_parameter_value().integer_array_value
        self.input = self.get_parameter(
            'input').get_parameter_value().bool_value
        self.line_count = self.get_parameter(
            'line_count').get_parameter_value().integer_value

    def publish(self, s):
        """Publish message on topic."""
        msg = String()
        msg.data = s
        self.pub.publish(msg)

    def create_input_subscriptions(self, callback, callback_cr):
        self.sub1 = self.create_subscription(
            String, 'input', callback, 1000)
        self.sub2 = self.create_subscription(
            String, 'input_cr', callback_cr, 1000)

# stdin thread class

class StdinThread(QtCore.QThread):
    """QThread class to handle stdin reading."""
    result = QtCore.pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.running = True

    def run(self):
        """Read stdin loop function."""
        flags = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)
        while self.running:
            rlist, w, e = select.select([sys.stdin], [], [])
            if rlist:
                try:
                    buffer = sys.stdin.buffer.read(128).decode()
                except IOError:
                    buffer = []
                if len(buffer):
                    self.result.emit(buffer)
            else: self.sleep(0.1)
        sys.stdin.close()

    def stop(self):
        """Mark thread to stop the loop."""
        self.running = False

# window class

class Window(QWidget):
    """Main Window with gpt4all terminal."""
    def __init__(self, node : RosNode):
        QWidget.__init__(self)
        self.node = node
        self.worker = StdinThread()
        self.worker.result.connect(self.update_text)
        self.oldPosition = QtCore.QPoint(0,0)

        # setup main window
        self.setWindowTitle(self.node.title)
        self.setStyleSheet(self.node.stylesheet_window)
        self.setWindowOpacity(self.node.opacity)
        if self.node.frameless:
            self.setWindowFlag(QtCore.Qt.FramelessWindowHint)
        if len(self.node.geometry) > 3:
            self.setGeometry(
                self.node.geometry[0], 
                self.node.geometry[1], 
                self.node.geometry[2], 
                self.node.geometry[3]) 

        # create plain text area
        self.tarea = QPlainTextEdit()
        f = QtGui.QFont(self.node.fontname)
        f.setPointSize(self.node.fontsize)
        self.tarea.setFont(f)
        self.tarea.setStyleSheet(self.node.stylesheet)
        self.tarea.setVerticalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        if self.node.line_count > 0:
            self.tarea.setMaximumBlockCount(self.node.line_count)

        # create input text field
        self.text = ''
        self.tedit = QLineEdit()
        self.tedit.setStyleSheet("QLineEdit { %s }" % self.node.stylesheet)
        self.tedit.textChanged.connect(self.input_changed)
        self.tedit.editingFinished.connect(self.input_enter)

        # put together layout
        layout = QGridLayout()
        if len(self.node.margin) > 3:
            layout.setContentsMargins(
                self.node.margin[0], 
                self.node.margin[1], 
                self.node.margin[2], 
                self.node.margin[3])
        layout.setSpacing(0)
        self.setLayout(layout)
        layout.addWidget(self.tarea)
        if self.node.input:
            layout.addWidget(self.tedit)
            self.tedit.setFocus()
        else: self.tarea.setFocus()

        if self.node.display >= 0:
            screen = QtGui.QGuiApplication.screens()[self.node.display]
            self.move(screen.geometry().x()+self.node.geometry[0], 
                      screen.geometry().y()+self.node.geometry[1],)

        self.node.create_input_subscriptions(self.input_callback, self.input_callback_cr)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(100)

    def spin_once(self):
        """Spin periodically once the ROS node."""
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def start_read_stdin(self):
        """Start the worker to read the incomming GPT generator output."""
        self.worker.start()

    def update_text(self, s : str):
        """Append given text in the PlainText area."""
        self.tarea.moveCursor(QtGui.QTextCursor.End)
        self.tarea.insertPlainText(s)
        self.tarea.moveCursor(QtGui.QTextCursor.End)

    def input_callback(self, msg : String):
        """Receive input text via ROS topic and update text."""
        self.update_text(msg.data)

    def input_callback_cr(self, msg : String):
        """Receive input text via ROS topic and update text with CR"""
        self.update_text(msg.data+"\n")

    def input_changed(self, text):
        """Input edit change text callback."""
        self.text = text

    def input_enter(self):
        """Input edit enter callback."""
        if self.text:
            self.node.publish(self.text)
        self.tedit.setText('')
        self.text = ''

    def closeEvent(self, event):
        """Window close callback. This stops also the stdin worker thread."""
        self.worker.stop()
        event.accept()
    
    def mousePressEvent(self, event):
        """Mouse press callback. Handles also window moving without caption."""
        self.oldPosition = event.globalPos()
        #if event.button() == QtCore.Qt.RightButton:
        #    self.close()

    def mouseMoveEvent(self, event):
        """Mouse move callback. Handles also window moving without caption."""
        delta = QtCore.QPoint(event.globalPos() - self.oldPosition)
        self.move(self.x() + delta.x(), self.y() + delta.y())
        self.oldPosition = event.globalPos()

# main

def sigint_handler(*args):
    QApplication.quit()

def main():
    signal.signal(signal.SIGINT, sigint_handler)
    app = QApplication(sys.argv)
    rclpy.init(args=sys.argv)

    n = RosNode()
    screen = Window(n)

    if not os.isatty(sys.stdin.fileno()):
        screen.start_read_stdin()

    screen.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
