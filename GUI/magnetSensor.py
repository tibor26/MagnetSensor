import sys
import serial
import serial.tools.list_ports
import math
import numpy as np
import csv

from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QFileDialog
from PyQt5.QtGui import QDoubleValidator, QIntValidator
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
import pyqtgraph as pg

from magnetSensorGui import Ui_MainWindow

import os
import json
from datetime import datetime
import time


class SerialThread(QThread):
    data_received = pyqtSignal(list)

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.buffer = []

    def run(self):
        try:
            with serial.Serial(self.port, self.baudrate, timeout=0.005) as ser:
                ser.reset_input_buffer()
                # To read 8 bytes, starting with 0xA5, status, Xl, Xh, Yl, Yh, Zl, Zh

                while self.running:
                    if ser.in_waiting > 8:
                        ser.reset_input_buffer()
                    if ser.in_waiting == 8:
                        raw_data = ser.read(8)
                        if raw_data[0] == 0xA5:
                            self.data_received.emit(list(raw_data))

        except serial.SerialException as e:
            print(f"Serial error: {e}")

    def stop(self):
        self.running = False
        self.buffer.clear()
        self.wait()


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()

        self.plotWidget_RightTop = None
        self.curve_xm = None
        self.curve_ym = None
        self.curve_zm = None

        self.setupUi(self)

        self.serial_thread = None
        self.plotCount = self.horizontalSlider_Plot.value()     # Initialize plot count from the slider value

        self.pushButton_Start.clicked.connect(self.start_reading)
        self.pushButton_Refresh.clicked.connect(self.populate_ports)

        self.horizontalSlider_Plot.valueChanged.connect(self.update_plot_count)  # Connect slider signal for plot count

        self.populate_ports()

        self.data_buffer = []

        # Initialize plot
        self.init_plot()

    def populate_ports(self):
        ports = serial.tools.list_ports.comports()
        self.comboBox_Port.clear()
        for port in ports:
            self.comboBox_Port.addItem(port.device)

    def start_reading(self):
        if self.serial_thread:
            return

        selected_port = self.comboBox_Port.currentText()
        if not selected_port:
            QMessageBox.warning(self, "Warning", "No COM port selected")
            return

        try:
            with serial.Serial(selected_port, 57600) as ser:
                ser.write(b'\x55\xAA')
                time.sleep(0.1)
        except serial.SerialException as e:
            QMessageBox.critical(self, "Error", f"Could not open port {selected_port}: {e}")
            return

        self.serial_thread = SerialThread(selected_port, 57600)
        self.serial_thread.data_received.connect(self.message_process)

        self.serial_thread.start()
        # self.pushButton_Start.setEnabled(False)

    def update_plot_count(self):
        value = self.horizontalSlider_Plot.value()
        self.plotCount = value

    def message_process(self, message):
        if message[1] == 0xFF:
            self.label_ID.setText("No Data!")
            return
        if message[1] == 0x01:
            # Acknowledgement, Message received from the PC.
            self.label_ID.setText("Sent!")
            return

        # magnet fields:
        xm = message[2] + message[3] * 256
        if xm > 32768:
            xm = xm - 65536
        ym = message[4] + message[5] * 256
        if ym > 32768:
            ym = ym - 65536
        zm = message[6] + message[7] * 256
        if zm > 32768:
            zm = zm - 65536

        self.data_buffer.append((xm, ym, zm))

        self.data_buffer = self.data_buffer[-1000:]     # To only keep 1000 samples
# Calculate the magnet angle
        if (zm >= 30000) and (xm < 20000) and (ym < 20000):     # Z axis high value, it means the magnet is flat
            pos = 4
        elif (xm < 20000) and (ym < 20000) and (zm < 20000):
            pos = 0
        else:                   # magnet is vertical positioned, calculate the x/y angle
            rad = math.atan2(xm, ym)
            angle = abs(math.degrees(rad))
            # to recognize 3 positions 0-45-90
            if angle < 20:
                pos = 1
            elif angle > 65:
                pos = 3
            else:
                pos = 2

        # update the label
        if pos == 0:
            self.label_ID.setText("No Magnet")
        else:
            self.label_ID.setText(f"ID# {pos:2d}")

        # Update the plot
        self.update_plot()

    def closeEvent(self, event):
        super().closeEvent(event)

    def init_plot(self):
        self.plotWidget_RightTop.setBackground('w')

        # Right Bottom Plot: xm, ym, zm
        self.curve_xm = self.plotWidget_RightTop.plot(pen=pg.mkPen('r', width=2), name='xm_plot')
        self.curve_ym = self.plotWidget_RightTop.plot(pen=pg.mkPen('g', width=2), name='ym_plot')
        self.curve_zm = self.plotWidget_RightTop.plot(pen=pg.mkPen('b', width=2), name='zm_plot')
        self.plotWidget_RightTop.addLegend()
        # self.plotWidget_RightTop.setRange(yRange=[0, 1])

        # Add legend explicitly
        legend = self.plotWidget_RightTop.addLegend()
        legend.setParentItem(self.plotWidget_RightTop.graphicsItem())  # Ensure legend is parented to plot
        legend.anchor((1, 2), (1, 1))  # Anchor to top right corner

        legend.addItem(self.curve_xm, 'xm')
        legend.addItem(self.curve_ym, 'ym')
        legend.addItem(self.curve_zm, 'zm')

    def update_plot(self):
        # Get the latest xxx entries from data_buffer if available, or all if less than 500
        start_index = max(0, len(self.data_buffer) - self.plotCount)
        m_data = self.data_buffer[start_index:]

        xm_m = [entry[0] for entry in m_data]
        ym_m = [entry[1] for entry in m_data]
        zm_m = [entry[2] for entry in m_data]

        self.curve_xm.setData(xm_m)
        self.curve_ym.setData(ym_m)
        self.curve_zm.setData(zm_m)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
