from contrast.environment import macro
from contrast.motors.Motor import expect_motors
from contrast.environment import env

import sys
import time
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from random import randint

from IPython import get_ipython

class SimpleWindow(QtWidgets.QDialog):
    def __init__(self, *args, parent=None, **kwargs):
        super().__init__(*args, parent, **kwargs)
        uic.loadUi("lightSTXM_main.ui", self)

        #self.setGeometry(300, 300, 200, 80)
        self.setWindowTitle('LightSTXM')
        #self.plot_widget.setBackground('w')
        
        self.x = list(range(100))  # 100 time points
        self.y = [randint(0,100) for _ in range(100)]
        self.pen = pg.mkPen(color=(255, 0, 0), width=15, style=QtCore.Qt.DashLine)
        self.data_line = self.plot_widget.plot(self.x, self.y, pen=self.pen, symbol='+', symbolSize=30, symbolBrush=('b'))
        self.plot_widget.setTitle("My test plot 1", color="b", size="30pt")
        styles = {'color':'r', 'font-size':'20px'}
        self.plot_widget.setLabel('left', 'Temperature (°C)', **styles)
        self.plot_widget.setLabel('bottom', 'Hour (H)', **styles)
        self.plot_widget.showGrid(x=True, y=True)
        
        
        hour = [1,2,3,4,5,6,7,8,9,10]
        temperature_1 = [30,32,34,32,33,31,29,32,35,45]
        temperature_2 = [50,35,44,22,38,32,27,38,32,44]
        self.plot_widget.plot(hour, temperature_1, "Sensor1", 'r')
        self.plot_widget.plot(hour, temperature_2, "Sensor2", 'b')
        
        #self.quit = QtWidgets.QPushButton('Close', self)
        #self.quit.setGeometry(10, 10, 60, 35)
        
        #self.quit.clicked.connect(lambda _: sw.close())
        self.pushButton.clicked.connect(self.button_test)
        
        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        
    def button_test(self):
        self.plot_widget.clear()
        
    def plot(self, x, y, plotname, color):
        pen = pg.mkPen(color=color)
        self.plot_widget.plot(x, y, name=plotname, pen=pen, symbol='+', symbolSize=30, symbolBrush=(color))
        
    def update_plot_data(self):
    
        self.x = self.x[1:]  # Remove the first y element.
        self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last.

        self.y = self.y[1:]  # Remove the first
        self.y.append( randint(0,100))  # Add a new random value.

        self.data_line.setData(self.x, self.y) 

class MotorPositionWindow(QtWidgets.QMainWindow):
    """Defines the elements of the motor postion window"""
    DATA_BUFFER = 1000
    DISPLAY_TIME = 10 # 'Visible' time slot in the plot widgets
    
    def __init__(self, *args, update_rate=50, **kwargs):
        super().__init__()
        self.setWindowTitle('Motor Positions')
        
        self.motors = args
        expect_motors(self.motors)
        self.update_rate = update_rate
        
        self.v_layout = QtWidgets.QVBoxLayout()
        self.motor_layout = {}
        
        
        self.plot_widgets = {}
        self.time_axis = {}
        self.plots = {}
        self.curve = {}
        self.data = {}
        self.motor_edits = {}
        self.motor_attrs = {}
        self.curr_pos_label1 = {}
        self.curr_pos_label2 = {}
        self.goto_label = {}
        self.goto_button = {}
        self.clear_button = {}
        
        for motor in self.motors:
            self.plot_widgets[motor.name] = pg.PlotWidget(title=motor.name)
            self.plot_widgets[motor.name].enableAutoRange('xy', True)
            self.time_axis[motor.name] = pg.DateAxisItem()
            self.plot_widgets[motor.name].setAxisItems({'bottom':self.time_axis[motor.name]})
            
            self.motor_layout[motor.name] = QtWidgets.QHBoxLayout()
            self.motor_attrs[motor.name] = QtWidgets.QVBoxLayout()
            self.motor_layout[motor.name].addLayout(self.motor_attrs[motor.name])
            
            self.curr_pos_label1[motor.name] = QtWidgets.QLabel("Current postion:")
            self.curr_pos_label2[motor.name] = QtWidgets.QLabel("_")
            self.goto_label[motor.name] = QtWidgets.QLabel("Go to:")
            self.motor_edits[motor.name] = QtWidgets.QLineEdit()
            self.motor_edits[motor.name].setValidator(QtGui.QDoubleValidator())
            self.goto_button[motor.name] = QtWidgets.QPushButton("Move")
            self.clear_button[motor.name] = QtWidgets.QPushButton("Clear Plot")
            
            self.motor_attrs[motor.name].addWidget(self.curr_pos_label1[motor.name])
            self.motor_attrs[motor.name].addWidget(self.curr_pos_label2[motor.name])
            self.motor_attrs[motor.name].addWidget(self.goto_label[motor.name])
            self.motor_attrs[motor.name].addWidget(self.motor_edits[motor.name])
            self.motor_attrs[motor.name].addWidget(self.goto_button[motor.name])
            self.motor_attrs[motor.name].addWidget(self.clear_button[motor.name])
            self.motor_attrs[motor.name].addStretch()
            
            self.motor_layout[motor.name].addWidget(self.plot_widgets[motor.name])
            self.v_layout.addLayout(self.motor_layout[motor.name])
            
            self.goto_button[motor.name].clicked.connect(lambda state, motor=motor: self._motor_goto(motor))
            self.clear_button[motor.name].clicked.connect(lambda state, motor=motor: self._clear_plot(motor))
        
        self.widget = QtWidgets.QWidget()
        self.widget.setLayout(self.v_layout)
        self.setCentralWidget(self.widget)
        
        for motor in self.motors:
            self.curve[motor.name] = self.plot_widgets[motor.name].plot(pen='y', symbol='o')
            self.data[motor.name] = np.array([], dtype=np.float32)
            #self.clear_plot_action = self.plots[motor.name].vb.menu.addAction('Clear Plot')
            
            self.clear_plot_action = QtWidgets.QAction("Clear Plot")
            self.plot_widgets[motor.name].addAction(self.clear_plot_action)
            self.clear_plot_action.triggered.connect(lambda checked, motor=motor: self.clear_plot(motor))
        
        self.ptr = 0
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        
    def _motor_goto(self, motor):
        motor.move(float(self.motor_edits[motor.name].text()))
    
    def _clear_plot(self, motor):
        self.timer.stop()
        self.data[motor.name] = np.array([], dtype=np.float32)
        self.timer.start(self.update_rate)

        
    def update(self):
        for motor in self.motors:
            self.curve[motor.name].setData(self.data[motor.name])
        # if self.ptr == 0:
        #     self.plot.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
        if self.ptr >= self.DATA_BUFFER:
            self.ptr = 0

        for motor in self.motors:
            pos = motor.position()
            self.curr_pos_label2[motor.name].setText(f'{pos:.3f}')
            self.data[motor.name] = np.append(self.data[motor.name], pos)
            if self.ptr > int(self.DISPLAY_TIME / (self.update_rate * 0.001)):
                pass
        self.ptr += 1

@macro
class MotPosWid(object):
    """
    Displays the motor widget window and substitutes the qt event loop ::

        motorwidget motor_name(s)
    """

    def __init__(self, *args, update_rate=50, **kwargs):
        self.motors = args
        expect_motors(self.motors)
        self.update_rate = update_rate
        
        ipython = get_ipython()
        ipython.magic('gui qt5')

        if not hasattr(env, 'widgets'):
            env.widgets = []
        env.widgets.append(self)
        
        self.app = pg.mkQApp("Plotting motor positions")
        if self.app is None:
            print('self.app was None')
            self.app = QtWidgets.QApplication(sys.argv)

    def run(self):
        self.window = MotorPositionWindow(*self.motors)
        self.window.show()
        
        try:
            print('Starting Qt event loop..')
            from IPython.lib.guisupport import start_event_loop_qt4
            start_event_loop_qt4(self.app)
        except ImportError:
            print('Could not import start_event_loop_qt4.')
            self.app.exec_()

        self.window.timer.start(self.window.update_rate)

class TestWindowWidget(object):
    """
    Displays a motor widget form. ::

        motorwidget motor_name(s)
    """
    
    def __init__(self, *args):
        self.motors = args
        expect_motors(self.motors)
        
        ipython = get_ipython()
        ipython.magic('gui qt5')
        
        self.app = QtCore.QCoreApplication.instance()
        if self.app is None:
            print('self.app was None')
            self.app = QtWidgets.QApplication(sys.argv)

    def run(self):
        for motor in self.motors:
            print(f'The {motor.name} position is {motor.position()}')
            
        print('About to show a window..')
        self.window = SimpleWindow()
        self.window.show()
        
        try:
            print('Starting Qt event loop..')
            from IPython.lib.guisupport import start_event_loop_qt4
            start_event_loop_qt4(self.app)
        except ImportError:
            print('Could not import start_event_loop_qt4.')
            self.app.exec_()
            
