from contrast.environment import macro, MacroSyntaxError
from contrast.motors.Motor import expect_motors
from contrast.motors import all_are_motors

import sys
import time
import numpy as np
from random import randint

import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import QCoreApplication, Signal, Slot

# from contrast.widgets.slots import print_scan_nr
from contrast.scans.AScan import QSoftwareScan
from ..environment import macro, env
from IPython import get_ipython

class QAScanWindow(QtWidgets.QMainWindow):
    """Defines the elements of the QAScan window"""
    DATA_BUFFER = 1000
    
    def __init__(self, *args, update_rate=50, **kwargs):
        super().__init__()
        self.setWindowTitle('QAScan')
        
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
        self.curr_scan_nr = {}
        self.curr_scan_limits = {}
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
            
            self.curr_scan_nr[motor.name] = QtWidgets.QLabel("Current scan #: ")
            self.curr_scan_limits[motor.name] = QtWidgets.QLabel("Scan limits: ")
            self.goto_label[motor.name] = QtWidgets.QLabel("Go to:")
            self.motor_edits[motor.name] = QtWidgets.QLineEdit()
            self.motor_edits[motor.name].setValidator(QtGui.QDoubleValidator())
            self.goto_button[motor.name] = QtWidgets.QPushButton("Move")
            self.clear_button[motor.name] = QtWidgets.QPushButton("Clear Plot")
            
            self.motor_attrs[motor.name].addWidget(self.curr_scan_nr[motor.name])
            self.motor_attrs[motor.name].addWidget(self.curr_scan_limits[motor.name])
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
class QAScan(QSoftwareScan):
    """
    Software scan one or more motors in parallel. ::

        ascan <motor1> <start> <stop> ... <intervals> <exp_time>
    """

    def __init__(self, *args, **kwargs):
        self.motors = []
        self.limits = []
        try:
            exposuretime = float(args[-1])
            self.intervals = int(args[-2])
            super(QAScan, self).__init__(exposuretime)
            for i in range(int((len(args) - 2) / 3)):
                self.motors.append(args[3 * i])
                self.limits.append(
                    [float(m) for m in args[3 * i + 1:3 * i + 3]])
            # duplicate of limits for display purposes
            self.motor_limits = { motor.name:limits for (motor, limits) in zip(self.motors, self.limits) }
            self.n_positions = self.intervals + 1
            assert all_are_motors(self.motors)
            assert (len(args) - 2) % 3 == 0
        except:
            raise MacroSyntaxError
        
        ipython = get_ipython()
        ipython.magic('gui qt5')

        if not hasattr(env, 'widgets'):
            env.widgets = []
        env.widgets.append(self)
        
        self.app = pg.mkQApp("Plotting qascan progress")
        if self.app is None:
            print('self.app was None')
            self.app = QtWidgets.QApplication(sys.argv)
        
        self.scan_started.connect(self.print_scan_nr)

        self.window = QAScanWindow(*self.motors)
        self.window.show()
        
        try:
            print('Starting Qt event loop..')
            from IPython.lib.guisupport import start_event_loop_qt4
            start_event_loop_qt4(self.app)
        except ImportError:
            print('Could not import start_event_loop_qt4.')
            self.app.exec_()

    @Slot(int)
    def print_scan_nr(self, nr):
        for motor in self.motors:
            self.window.curr_scan_nr[motor.name].setText(f'Current scan #: {nr}')
            self.window.curr_scan_limits[motor.name].setText(f'Current limits: {self.motor_limits[motor.name]}')

    def _generate_positions(self):
        positions = []
        for i in range(len(self.motors)):
            positions.append(np.linspace(self.limits[i][0],
                                         self.limits[i][1],
                                         self.intervals + 1))
        for i in range(len(positions[0])):
            yield {m.name: pos[i] for (m, pos) in zip(self.motors, positions)}
