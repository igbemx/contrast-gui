import sys
import json
from collections import OrderedDict
import numpy as np
import logging
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic
from PyQt5.QtCore import Qt, QSettings
from PyQt5.QtCore import QThread, QTimer, pyqtSignal, pyqtSlot, QEventLoop
from PyQt5.QtGui import QDoubleValidator, QIntValidator
import pyqtgraph as pg
import pyqtgraph.parametertree as pt

from softimax_beamline import *
from macros import LineScan, QAScan, QMesh
from contrast.motors import Motor
from contrast.detectors.TangoAttributeDetector import TangoAttributeDetector
from contrast.recorders import RecorderHeader, RecorderFooter


class MotorUpdateThread(QThread):
      
    motor_pos_sig = pyqtSignal(OrderedDict)

    def __init__(self, *args, update_rate=500, **kwargs):
        QThread.__init__(self, *args, **kwargs)
        self.update_rate = update_rate
        self.motors = {m.name : m for m in Motor.getinstances()}
        self.motor_positions = OrderedDict()
        
        self.motorUpdateTimer = QTimer()
        self.motorUpdateTimer.moveToThread(self)
        self.motorUpdateTimer.timeout.connect(self.get_motor_pos)
        
    def get_motor_pos(self):
        """Gets motor positions and emits the values as a dict"""
        self.motor_positions = {m.name:m.position() for m in Motor.getinstances()}
        self.motor_positions = OrderedDict(sorted(self.motor_positions.items())) 
        self.motor_pos_sig.emit(self.motor_positions)

    def run(self):
        self.motorUpdateTimer.start(self.update_rate)
        loop = QEventLoop()
        loop.exec_()
        
class DetUpdateThread(QThread):
      
    det_value_sig = pyqtSignal(OrderedDict)

    def __init__(self, *args, update_rate=500, **kwargs):
        QThread.__init__(self, *args, **kwargs)
        self.update_rate = update_rate
        self.detectors = {d.name : d for d in TangoAttributeDetector.getinstances()}
        self.det_values = OrderedDict()
        
        self.detUpdateTimer = QTimer()
        self.detUpdateTimer.moveToThread(self)
        self.detUpdateTimer.timeout.connect(self.get_det_val)
        
    def get_det_val(self):
        """Gets current 0D detector values and emits them as a dict"""
        self.det_values = {d.name:d.read() for d in TangoAttributeDetector.getinstances()}
        self.det_values = OrderedDict(sorted(self.det_values.items())) 
        self.det_value_sig.emit(self.det_values)
        
    @pyqtSlot(int)
    def _start_timer(self, update_rate):
        self.detUpdateTimer.start(update_rate)
        
    @pyqtSlot()
    def _stop_timer(self):
        self.detUpdateTimer.stop()

    def run(self):
        self._start_timer(self.update_rate)
        loop = QEventLoop()
        loop.exec_()


class ContrastGUIMainWindow(QtWidgets.QDialog):
    det_timer_start_sig = pyqtSignal(int)
    det_timer_stop_sig = pyqtSignal()
    ls_macro_cancel_sig = pyqtSignal()
      
    def __init__(self, *args, env=None, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi("STXM_main.ui", self)
        
        self.ls_settings = QSettings('ContrastGUI', 'LineScan')
        self.ms_settings = QSettings('ContrastGUI', 'MeshScan')
        logging.info(f"Line scan settings are stored in: {self.ls_settings.fileName()}")
        
        self.env = env
        
        self.available_motors = {m.name:m for m in Motor.getinstances()}
        self.available_detectors = {d.name:d for d in TangoAttributeDetector.getinstances()}
        self.all_detectors = {d.name:d for d in Detector.getinstances()}
        self.available_motors = OrderedDict(sorted(self.available_motors.items()))
        self.available_detectors = OrderedDict(sorted(self.available_detectors.items()))
        
        self._populate_det_combo()
        self._current_det = self.det_combo.currentText()
        self.det_combo.currentTextChanged.connect(self._select_det_combo)
        
        self._populate_ls_det_par_combo() # Populating the detecotr parameter combo (line scan)
        self._select_det_par_combo()
        
        self._populate_ls_mot_combo()
        self._populate_ls_det_combo()
        self._populate_ms_det_combo()
        self._populate_ms_fast_mot_combo()
        self._populate_ms_slow_mot_combo()
        self._populate_ms_hor_axis_combo()
        self._populate_ms_vert_axis_combo()
        
        self.ls_mot_sel_combo.currentTextChanged.connect(self._select_ls_mot_combo)
        self.ls_det_sel_combo.currentTextChanged.connect(self._select_ls_det_combo)
        
        self.ms_det_sel_combo.currentTextChanged.connect(self._select_ms_det_combo)
        self.ms_fast_mot_sel_combo.currentTextChanged.connect(self._select_ms_fmot_combo)
        self.ms_slow_mot_sel_combo.currentTextChanged.connect(self._select_ms_smot_combo)
        
        self._ms_current_hor_axis = self.ms_hor_axis_combo.currentText()
        self._ms_current_vert_axis = self.ms_vert_axis_combo.currentText()
        self.ms_hor_axis_combo.currentTextChanged.connect(self._select_ms_hor_axis)
        self.ms_vert_axis_combo.currentTextChanged.connect(self._select_ms_vert_axis)
        
        self._current_ls_det_par = self.ls_det_par_combo.currentText()
        self.ls_det_par_combo.currentTextChanged.connect(self._select_det_par_combo)
        
        
        # Setting up Line Scan validators
        self.ls_start_edit.setValidator(QDoubleValidator())
        self.ls_stop_edit.setValidator(QDoubleValidator())
        self.ls_dwell_edit.setValidator(QDoubleValidator())
        self.ls_points_n_edit.setValidator(QIntValidator())
        
        # Setting up Mesh Scan validators
        self.ms_fast_start_edit.setValidator(QDoubleValidator())
        self.ms_fast_stop_edit.setValidator(QDoubleValidator())
        self.ms_dwell_edit.setValidator(QDoubleValidator())
        self.ms_fast_points_n_edit.setValidator(QIntValidator())
        
        self.ms_slow_start_edit.setValidator(QDoubleValidator())
        self.ms_slow_stop_edit.setValidator(QDoubleValidator())
        self.ms_slow_points_n_edit.setValidator(QIntValidator())
        
        # Setting up Line Scan tab
        self.ls_macro_start_btn.setEnabled(True)
        self.ls_macro_start_btn.setDown(False)
        self.ls_curves = OrderedDict()
        self._cur_ls_plot = OrderedDict()
        
        # Setting up Mesh Scan tab - START
        # ==============================================================
        self.ms_show_roi_cb.setCheckState(False)
        self.ms_show_roi_cb.stateChanged.connect(self._show_roi_cb_changed)
        
        pg.setConfigOption('imageAxisOrder', 'row-major') # best performance
        
        self.ms_images = OrderedDict()
        self.ms_img_data = OrderedDict()
        self.ms_data = OrderedDict()
        self.ms_plot = self.mesh_scan_widget.addPlot(title="Mesh Scan", enableMenu=True) # Returns PlotItem
        # Context menu actions
        self.ms_plot.vb.menu = QtWidgets.QMenu(self.mesh_scan_widget)
        self.ms_test_action = QtWidgets.QAction("Future Action :)", checkable=True)
        self.ms_plot.vb.menu.addAction(self.ms_test_action)
        self.ms_test_action.toggled.connect(lambda state: print('Test action, state:', state))
        
        
        # Contrast/color control
        self.hist = pg.HistogramLUTItem()
        self.mesh_scan_widget.addItem(self.hist)
        self.hist.vb.menu = QtWidgets.QMenu()
        self.ms_autoz_action = QtWidgets.QAction("Auto Scale", checkable=True)
        self.hist.vb.menu.addAction(self.ms_autoz_action)
        self._ms_hist_auto_levels = True
        self.ms_autoz_action.setChecked(True)
        self.ms_autoz_action.toggled.connect(self._change_ms_autoz_state)
        
        # Setting up Mesh Scan tab - END
        # ==============================================================
        
        # Motor table related things
        self.motor_table.setRowCount(10)
        self.motor_table.setColumnCount(4)
        self.motor_table.setHorizontalHeaderLabels(['Motor', 'Position', 'GoTo', 'State'])
        self.motor_table.itemChanged.connect(self.move_motor)
        
        # Setting up 0D plot axis
        self.time_axis = pg.DateAxisItem()
        self.plot_widget.setAxisItems({'bottom':self.time_axis})
        self.det_curve = self.plot_widget.plot(pen='y', symbol='o')
        # Detector plot related things
        self.det_plot_data = np.array([], dtype=np.float32)
        
        # Starting motor table update thread
        self.mot_upd_thread = MotorUpdateThread(update_rate=500)
        self.mot_upd_thread.start()
        self.mot_upd_thread.motor_pos_sig.connect(self.update_motor_table)
        # Starting detectors plot update thread
        self.det_upd_thread = DetUpdateThread(update_rate=500)
        self.det_upd_thread.start()
        self.det_upd_thread.det_value_sig.connect(self.update_0D_det_plot)
        
        # Macro related things
        self.ls_macro_start_btn.clicked.connect(self._ls_macro_start_btn)
        self.ls_macro_cancel_btn.clicked.connect(self._ls_macro_cancel_btn)
        self.ms_macro_cancel_btn.clicked.connect(self._ls_macro_cancel_btn)
        
        self.ms_macro_start_btn.clicked.connect(self._ms_macro_start_btn)
        self.ms_macro_cancel_btn.clicked.connect(self._ms_macro_cancel_btn)
        
        # Auxillary signals wiring
        self.det_timer_start_sig.connect(self.det_upd_thread._start_timer)
        self.det_timer_stop_sig.connect(self.det_upd_thread._stop_timer)
        
        try:
          self.ls_mot_sel_combo.setCurrentText(self.ls_settings.value('ls_mot_sel_combo'))
          self.ls_det_sel_combo.setCurrentText(self.ls_settings.value('ls_det_sel_combo'))
          self.ls_det_par_combo.setCurrentText(self.ls_settings.value('ls_det_par_combo'))
          self.ls_start_edit.setText(self.ls_settings.value('ls_start_edit'))
          self.ls_stop_edit.setText(self.ls_settings.value('ls_stop_edit'))
          self.ls_points_n_edit.setText(self.ls_settings.value('ls_points_n_edit'))
          self.ls_dwell_edit.setText(self.ls_settings.value('ls_dwell_edit'))
          self._select_ls_mot_combo()
          self._select_ls_det_combo()
          self._select_det_par_combo()
          
          self.ms_fast_mot_sel_combo.setCurrentText(self.ms_settings.value('ms_fast_mot_sel_combo'))
          self.ms_fast_start_edit.setText(self.ms_settings.value('ms_fast_start_edit'))
          self.ms_fast_stop_edit.setText(self.ms_settings.value('ms_fast_stop_edit'))
          self.ms_fast_points_n_edit.setText(self.ms_settings.value('ms_fast_points_n_edit'))
          self.ms_slow_mot_sel_combo.setCurrentText(self.ms_settings.value('ms_slow_mot_sel_combo'))
          self.ms_slow_start_edit.setText(self.ms_settings.value('ms_slow_start_edit'))
          self.ms_slow_stop_edit.setText(self.ms_settings.value('ms_slow_stop_edit'))
          self.ms_slow_points_n_edit.setText(self.ms_settings.value('ms_slow_points_n_edit'))
          self.ms_hor_axis_combo.setCurrentText(self.ms_settings.value('ms_hor_axis_combo'))
          self.ms_vert_axis_combo.setCurrentText(self.ms_settings.value('ms_vert_axis_combo'))
          self.ms_det_sel_combo.setCurrentText(self.ms_settings.value('ms_det_sel_combo'))
          self.ms_dwell_edit.setText(self.ms_settings.value('ms_dwell_edit'))
          self.ms_keep_img_cb.setChecked(self.ms_settings.value('ms_keep_img_cb'))
          self._select_ms_fmot_combo()
          self._select_ms_smot_combo()
          self._select_ms_hor_axis()
          self._select_ms_vert_axis()
          
        except Exception as e:
          logging.info('An exception during settings import: {e}')
    
    def _populate_det_combo(self):
        for name in self.available_detectors:
            self.det_combo.addItem(name)
            
    def _populate_ls_mot_combo(self):
        for name in self.available_motors:
            self.ls_mot_sel_combo.addItem(name)
            
    def _populate_ls_det_combo(self):
        for name in self.available_detectors:
            self.ls_det_sel_combo.addItem(name)
            
    def _populate_ls_det_par_combo(self):
        for d in Detector.getinstances():
            self.ls_det_par_combo.addItem(d.name)
              
    def _populate_ms_fast_mot_combo(self):
        for name in self.available_motors:
            self.ms_fast_mot_sel_combo.addItem(name)
            
    def _populate_ms_slow_mot_combo(self):
        for name in self.available_motors:
            self.ms_slow_mot_sel_combo.addItem(name)
            
    def _populate_ms_hor_axis_combo(self):
        for name in self.available_motors:
            self.ms_hor_axis_combo.addItem(name)
            
    def _populate_ms_vert_axis_combo(self):
        for name in self.available_motors:
            self.ms_vert_axis_combo.addItem(name)

    def _populate_ms_det_combo(self):
        for name in self.available_detectors:
            self.ms_det_sel_combo.addItem(name)       
    
    def _select_det_combo(self):
        self._current_det = self.det_combo.currentText()
        self.det_plot_data = np.array([], dtype=np.float32)
        
    def _select_ls_mot_combo(self):
        self._ls_current_mot = self.ls_mot_sel_combo.currentText()
        if hasattr(self, '_scan_data'):
          self._update_ls_plot()
      
    def _select_ls_det_combo(self):
        self._ls_current_det = self.ls_det_sel_combo.currentText()
        if hasattr(self, '_scan_data'):
          self.line_scan_widget.clear()
          self._cur_ls_plot[self._curr_header['scannr']] = self.line_scan_widget.plot(pen='y', symbol='o')
          self._update_ls_plot()
    
    def _select_det_par_combo(self):
        self._current_ls_det_par = self.ls_det_par_combo.currentText()
        self._cur_det_params = []
        curr_det = self.all_detectors[self._current_ls_det_par]
        for k,v in curr_det.__dict__.items():
            if isinstance(v, (float, int, str, list, dict, tuple)):
                self._cur_det_params.append({'name':k, 'type':type(v).__name__, 'value':v})
            else:
                self._cur_det_params.append({'name':k, 'type':'str', 'value':str(v)})
        
        param = pt.Parameter.create(name='params', type='group', children=self._cur_det_params)
        self.det_par_widget.setParameters(param)
        
    def _select_ms_det_combo(self):
        self._ms_current_det = self.ms_det_sel_combo.currentText()
        
    def _select_ms_fmot_combo(self):
        self._ms_current_fast_mot = self.ms_fast_mot_sel_combo.currentText()
    
    def _select_ms_smot_combo(self):
        self._ms_current_slow_mot = self.ms_slow_mot_sel_combo.currentText()
        
    def _select_ms_hor_axis(self):
        self._ms_current_hor_axis = self.ms_hor_axis_combo.currentText()
        if hasattr(self, '_motor_indices'):
          self._hor_mot_indx = self._motor_indices[self._ms_current_hor_axis]
          
    def _select_ms_vert_axis(self):
        self._ms_current_vert_axis = self.ms_vert_axis_combo.currentText()
        if hasattr(self, '_motor_indices'):
          self._vert_mot_indx = self._motor_indices[self._ms_current_vert_axis]
    
    def _show_roi_cb_changed(self, int):
        if self.ms_show_roi_cb.isChecked():
          self.roi = pg.ROI([-8, 14], [6, 5], removable=False)
          self.roi.addScaleHandle([0.5, 1], [0.5, 0.5])
          self.roi.addScaleHandle([0, 0.5], [0.5, 0.5])
          try:
            hor_pos = self.desc['limits'][self._motor_indices[self._ms_current_hor_axis]][0]
            vert_pos = self.desc['limits'][self._motor_indices[self._ms_current_vert_axis]][0]
            self.roi.setPos(hor_pos, vert_pos)
            self.roi.menu = QtWidgets.QMenu()
            self.ms_get_roi_rect_act = QtWidgets.QAction("Get ROI coordinates")
            self.roi.menu.addAction(self.ms_get_roi_rect_act)
            self.ms_get_roi_rect_act.triggered.connect(self._ms_roi_get_rect)
          except Exception:
            logging.info("No data for the ROI.")
          self.roi.removable = True
          self.ms_plot.addItem(self.roi)
          self.roi.setZValue(10)  # make sure ROI is drawn above image
        else:
          self.ms_plot.removeItem(self.roi)
          
    def _ms_roi_get_rect(self):
        roi_x, roi_y = self.roi.pos()
        roi_w, roi_h = self.roi.size()
        logging.info(f'Roi position is: {roi_x, roi_y}, the size is: {roi_w, roi_h}.')
        
        if self._motor_indices[self._ms_current_hor_axis] == self._motor_indices[self._ms_current_fast_mot]:
              self.ms_fast_start_edit.setText(f'{roi_x:.3f}')
              self.ms_slow_start_edit.setText(f'{roi_y:.3f}')
              self.ms_fast_stop_edit.setText(f'{(roi_x + roi_w):.3f}')
              self.ms_slow_stop_edit.setText(f'{(roi_y + roi_h):.3f}')
        elif self._motor_indices[self._ms_current_hor_axis] == self._motor_indices[self._ms_current_slow_mot]:
              self.ms_fast_start_edit.setText(f'{roi_y:.3f}')
              self.ms_slow_start_edit.setText(f'{roi_x:.3f}')
              self.ms_fast_stop_edit.setText(f'{(roi_y + roi_h):.3f}')
              self.ms_slow_stop_edit.setText(f'{(roi_x + roi_w):.3f}')
    
    def _change_ms_autoz_state(self, state):
        self._ms_hist_auto_levels = state
        
    @pyqtSlot(OrderedDict)
    def update_0D_det_plot(self, det_values):
        # print(det_values[self._current_det])
        self.det_plot_data = np.append(self.det_plot_data, det_values[self._current_det])
        self.det_curve.setData(self.det_plot_data)
        
    def _clear_det_plot(self):
        self.det_timer_stop_sig.emit()
        self.det_plot_data = np.array([], dtype=np.float32)
        self.det_timer_start_sig.start(self.update_rate)
        
    @pyqtSlot(OrderedDict)
    def update_motor_table(self, motor_positions):
        i = 0
        self.last_mot_positions = motor_positions
        for k, v in motor_positions.items():
              self.motor_table.setItem(i, 0, QtWidgets.QTableWidgetItem(k))
              self.motor_table.setItem(i, 1, QtWidgets.QTableWidgetItem(f'{v:.3f}'))
              i += 1
              
    def move_motor(self, item):
          if item.column() == 2:
            table_motors = list(self.last_mot_positions.items())
            motor_name = table_motors[item.row()][0]
            cell_value = float(self.motor_table.item(item.row(), item.column()).text())
            self.available_motors[motor_name].move(cell_value)
            
    def run_macro(self, params, Macro, env=None):
        logging.info(f"Run macro params: {params}")
        # Create a QThread object
        self.macro_thread = QThread()
        logging.info(f"Macro QThread created.")
        # Create a worker object
        self.macro_worker = Macro(*params, env=env)
        # Move worker to the thread
        self.macro_worker.moveToThread(self.macro_thread)
        # Connect signals and slots
        self.macro_thread.started.connect(self.macro_worker.run)
        self.macro_worker.scan_finished.connect(self.macro_thread.quit)
        self.macro_worker.scan_finished.connect(self.macro_worker.deleteLater)
        self.macro_thread.finished.connect(self.macro_thread.deleteLater)
        self.macro_worker.scan_started.connect(self.scan_started)
        self.macro_worker.scan_data.connect(self.scan_data)
        self.macro_worker.progress.connect(self.display_progress)
        self.ls_macro_cancel_sig.connect(self.macro_worker.macro_cancel)
        # Start the thread
        self.macro_thread.start()

        # Final resets
        self.ls_macro_start_btn.setEnabled(False)
        self.macro_worker.scan_finished.connect(
            lambda: self.ls_macro_start_btn.setEnabled(True)
        )
        self.ms_macro_start_btn.setEnabled(False)
        self.macro_worker.scan_finished.connect(
            lambda: self.ms_macro_start_btn.setEnabled(True)
        )
    
    def _ls_macro_start_btn(self):
          self._ls_current_mot = self.ls_mot_sel_combo.currentText()
          self._ls_current_det = self.ls_det_sel_combo.currentText()
          self._ls_current_start = self.ls_start_edit.text()
          self._ls_current_stop = self.ls_stop_edit.text()
          self._ls_current_points_n = self.ls_points_n_edit.text()
          self._ls_current_dwell = self.ls_dwell_edit.text()
          
          self.ls_macro_params = {
            'motor' : self.available_motors[self._ls_current_mot],
            'detector' : self.available_detectors[self._ls_current_det],
            'start' : float(self._ls_current_start),
            'stop' : float(self._ls_current_stop),
            'points_n' : int(self._ls_current_points_n),
            'dwell' : float(self._ls_current_dwell)
          }
          
          # Preparing params according to the AScan order
          ascan_params = [
            self.ls_macro_params['motor'],
            self.ls_macro_params['start'],
            self.ls_macro_params['stop'],
            self.ls_macro_params['points_n']-1,
            self.ls_macro_params['dwell']
          ]
          
          logging.info(f"New parameters acquired {ascan_params}.")
          
          self.run_macro(
            ascan_params,
            Macro=QAScan,
            env=self.env
          )
    
    def _ms_macro_start_btn(self):
        self._ms_current_fast_mot = self.ms_fast_mot_sel_combo.currentText()
        self._ms_current_slow_mot = self.ms_slow_mot_sel_combo.currentText()
        self._ms_current_fast_start = self.ms_fast_start_edit.text()
        self._ms_current_fast_stop = self.ms_fast_stop_edit.text()
        self._ms_current_slow_start = self.ms_slow_start_edit.text()
        self._ms_current_slow_stop = self.ms_slow_stop_edit.text()
        self._ms_current_fast_points_n = self.ms_fast_points_n_edit.text()
        self._ms_current_slow_points_n = self.ms_slow_points_n_edit.text()
        self._ms_current_det = self.ms_det_sel_combo.currentText()
        self._ms_current_dwell = self.ms_dwell_edit.text() 
        
        self.ms_macro_params = {
            'motor_fast' : self.available_motors[self._ms_current_fast_mot],
            'motor_slow' : self.available_motors[self._ms_current_slow_mot],
            'start_fast' : float(self._ms_current_fast_start),
            'stop_fast' : float(self._ms_current_fast_stop),
            'start_slow' : float(self._ms_current_slow_start),
            'stop_slow' : float(self._ms_current_slow_stop),
            'points_n_fast' : int(self._ms_current_fast_points_n),
            'points_n_slow' : int(self._ms_current_slow_points_n),
            'detector' : self.available_detectors[self._ms_current_det],
            'dwell' : float(self._ms_current_dwell)
          }
        
        ms_scan_params = [
            self.ms_macro_params['motor_slow'],
            self.ms_macro_params['start_slow'],
            self.ms_macro_params['stop_slow'],
            self.ms_macro_params['points_n_slow']-1,
            self.ms_macro_params['motor_fast'],
            self.ms_macro_params['start_fast'],
            self.ms_macro_params['stop_fast'],
            self.ms_macro_params['points_n_fast']-1,
            self.ms_macro_params['dwell']
          ]
        print(self.env.snapshot.capture())
        self.run_macro(
            ms_scan_params,
            Macro=QMesh,
            env=self.env
          )
    def _ms_macro_cancel_btn(self):
        pass
          
    def _ls_macro_cancel_btn(self):
        logging.info(f"Emitting ls_macro_cancel_sig")
        self.ls_macro_cancel_sig.emit()
    
    def display_progress(self, iter):
        pass
    
    def _update_ls_plot(self):
        self._cur_ls_plot[self._curr_header['scannr']].setData(
          x=self.ls_curves[self._curr_header['scannr']][self._ls_current_mot],
          y=self.ls_curves[self._curr_header['scannr']][self._ls_current_det],
          clear=True
        )
        
    def _update_ms_plot(self):
        pass
         
    # def mousePressEvent(self, QMouseEvent):
    #     if QMouseEvent.button() == Qt.LeftButton:
    #         print("Left Button Clicked")
    #     elif QMouseEvent.button() == Qt.RightButton:
    #         print("Right Button Clicked")
    #         menu = self.ms_plot.getMenu()
    #         pos  = QMouseEvent.screenPos()
    #         menu.popup(QtCore.QPoint(pos.x(), pos.y()))
    #         logging.info(f"menu is: {menu}.")
    
    # def contextMenuEvent(self, event):
    #     # contextMenu = QtWidgets.QMenu(self)
    #     # auto_z_scale = QtWidgets.QAction("Auto Z Scale", self, checkable=True)
    #     # newAct = contextMenu.addAction()
        
    #     action = self.ms_context_menu.exec_(self.mapToGlobal(event.pos()))
    #     if action == self.ms_autoz_action:
    #         pass
    
    
    @pyqtSlot(RecorderHeader)
    def scan_started(self, header):
        self._curr_header = header
        self.desc = json.loads(header['description'])
        self._scan_data = OrderedDict()
        logging.info(f"Scan {self.desc} started.")
        if self.desc['scan'] == 'ascan':
          logging.info(f"Craeating 2D plot for ascan (Line Scan), scannr is: {header['scannr']}")
          self.ls_curves[header['scannr']] = OrderedDict()
          self.line_scan_widget.clear()
          self._cur_ls_plot[self._curr_header['scannr']] = self.line_scan_widget.plot(pen='y', symbol='o')
        elif self.desc['scan'] == 'mesh':
          logging.info(f"Craeating 2D plot for mesh (Mesh Scan), scannr is: {header['scannr']}")
          logging.info(f"self.desc is: {self.desc}")
          try:
            if not self.ms_keep_img_cb.isChecked():
              self.ms_plot.removeItem(self.ms_images[header['scannr']-1]) # Remove previous image from the plot
          except Exception as e:
            logging.info(f"There was no previous image.")
          self.ms_data[header['scannr']] = OrderedDict()
          
          self.ms_images[header['scannr']] = pg.ImageItem()
          
          self._motor_indices = {m:i for i, m in enumerate(self.desc['motors'])} 
          self.ms_img_data[header['scannr']] = np.zeros(shape=(self.desc['points'][self._motor_indices[self._ms_current_slow_mot]], self.desc['points'][self._motor_indices[self._ms_current_fast_mot]]), dtype=np.double)
          
          
          self._ms_hor_range = abs(self.desc['limits'][self._motor_indices[self._ms_current_hor_axis]][1] - self.desc['limits'][self._motor_indices[self._ms_current_hor_axis]][0])
          self._ms_vert_range = abs(self.desc['limits'][self._motor_indices[self._ms_current_vert_axis]][1] - self.desc['limits'][self._motor_indices[self._ms_current_vert_axis]][0])
          self._mpp_coef_vert = self._ms_vert_range / self.desc['points'][self._motor_indices[self._ms_current_vert_axis]] # microns per point
          self._mpp_coef_hor = self._ms_hor_range / self.desc['points'][self._motor_indices[self._ms_current_hor_axis]]
          
          self._slow_pnt_indx, self._fast_pnt_indx = 0, 0
          # self._motor_indices[self._ms_current_slow_mot]
          
          self._tr = QtGui.QTransform()
          self.ms_images[header['scannr']].setTransform(self._tr.scale(self._mpp_coef_hor, self._mpp_coef_vert)
                                                        .translate((self.desc['limits'][self._motor_indices[self._ms_current_hor_axis]][0] / self._mpp_coef_hor), (self.desc['limits'][self._motor_indices[self._ms_current_vert_axis]][0] / self._mpp_coef_vert)))
          
          self.ms_plot.addItem(self.ms_images[header['scannr']])
          self.hist.setImageItem(self.ms_images[header['scannr']])
          self.ms_images[header['scannr']].setImage(self.ms_img_data[header['scannr']])
          
    @pyqtSlot(OrderedDict)
    def scan_data(self, data):
        logging.info(f'Received data is: {data}')
        for k, v in data.items():
            if k in self._scan_data:
              self._scan_data[k].append(v)
            else:
              self._scan_data[k] = [v]
        if self.desc['scan'] == 'ascan':
          self.ls_curves[self._curr_header['scannr']].update(self._scan_data)
          logging.info(f"self.ls_curves is: {self.ls_curves}.")
          self._update_ls_plot()
        elif self.desc['scan'] == 'mesh':          
          self.ms_images[self._curr_header['scannr']].setImage(
            self.ms_img_data[self._curr_header['scannr']],
            autoLevels = False
          )
          if self._motor_indices[self._ms_current_fast_mot] == self._motor_indices[self._ms_current_hor_axis]:
                self.ms_img_data[self._curr_header['scannr']][self._slow_pnt_indx][self._fast_pnt_indx] = data[self._ms_current_det]
          elif self._motor_indices[self._ms_current_fast_mot] == self._motor_indices[self._ms_current_vert_axis]:
                self.ms_img_data[self._curr_header['scannr']][self._fast_pnt_indx][self._slow_pnt_indx] = data[self._ms_current_det]
          
          # Keeping track of point index
          self._fast_pnt_indx += 1
          if self._fast_pnt_indx == (self.ms_macro_params['points_n_fast']):
              self._fast_pnt_indx = 0
              self._slow_pnt_indx += 1
              
          min_val = np.min(self.ms_img_data[self._curr_header['scannr']][np.nonzero(self.ms_img_data[self._curr_header['scannr']])])
          max_val = np.max(self.ms_img_data[self._curr_header['scannr']][np.nonzero(self.ms_img_data[self._curr_header['scannr']])])
          if self._ms_hist_auto_levels:    
            self.hist.setLevels(min_val, max_val)
            self.hist.setHistogramRange(min_val, max_val)
          else:
            pass

    @pyqtSlot(RecorderFooter)
    def scan_finished(self, footer):
        desc = json.loads(footer['description'])
        logging.info(f"Scan {desc} finished.")

        
    def closeEvent(self, event):
        self.ls_settings.setValue('ls_mot_sel_combo', self.ls_mot_sel_combo.currentText())
        self.ls_settings.setValue('ls_det_sel_combo', self.ls_det_sel_combo.currentText())
        self.ls_settings.setValue('ls_det_par_combo', self.ls_det_par_combo.currentText())
        self.ls_settings.setValue('ls_start_edit', self.ls_start_edit.text())
        self.ls_settings.setValue('ls_stop_edit', self.ls_stop_edit.text())
        self.ls_settings.setValue('ls_points_n_edit', self.ls_points_n_edit.text())
        self.ls_settings.setValue('ls_dwell_edit', self.ls_dwell_edit.text())
        
        self.ms_settings.setValue('ms_keep_img_cb', self.ms_keep_img_cb.isChecked())
        
        self.ms_settings.setValue('ms_fast_mot_sel_combo', self.ms_fast_mot_sel_combo.currentText())
        self.ms_settings.setValue('ms_fast_start_edit', self.ms_fast_start_edit.text())
        self.ms_settings.setValue('ms_fast_stop_edit', self.ms_fast_stop_edit.text())
        self.ms_settings.setValue('ms_fast_points_n_edit', self.ms_fast_points_n_edit.text())
        self.ms_settings.setValue('ms_slow_mot_sel_combo', self.ms_slow_mot_sel_combo.currentText())
        self.ms_settings.setValue('ms_slow_start_edit', self.ms_slow_start_edit.text())
        self.ms_settings.setValue('ms_slow_stop_edit', self.ms_slow_stop_edit.text())
        self.ms_settings.setValue('ms_slow_points_n_edit', self.ms_slow_points_n_edit.text())
        
        self.ms_settings.setValue('ms_hor_axis_combo', self.ms_hor_axis_combo.currentText())
        self.ms_settings.setValue('ms_vert_axis_combo', self.ms_vert_axis_combo.currentText())
        self.ms_settings.setValue('ms_det_sel_combo', self.ms_det_sel_combo.currentText())
        self.ms_settings.setValue('ms_dwell_edit', self.ms_dwell_edit.text())
        


app = QtWidgets.QApplication(sys.argv)
window = ContrastGUIMainWindow(env=env)
window.show()
app.exec_()