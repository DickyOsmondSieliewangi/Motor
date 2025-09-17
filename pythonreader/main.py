import sys
import asset.resources 
from PyQt5 import QtWidgets, uic, QtChart, QtCore, QtGui
import struct
import numpy as np
import time
import serial.tools.list_ports
import matplotlib.pyplot as plt

def get_available_ports():
    """Return a list of available serial ports."""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

class RefreshingComboBox(QtWidgets.QComboBox):
    """QComboBox that refreshes its items whenever opened."""
    def showPopup(self):
        self.refresh_ports()
        super().showPopup()  # call parent to actually open dropdown

    def refresh_ports(self):
        ports = get_available_ports()

        current = self.currentText()
        self.clear()

        # Add default "no selection" item first
        self.addItem("Select a port…")

        if ports:
            self.addItems(ports)
            # Restore selection if still valid
            if current in ports:
                index = self.findText(current)
                self.setCurrentIndex(index)
            else:
                # reset back to default
                self.setCurrentIndex(0)
        else:
            self.addItem("No ports available")
            self.setCurrentIndex(0)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("ui/main.ui", self)  # load your main.ui file

        self.serial_conn = None  
        self.child_windows = {}

        self.olc.clicked.connect(self.olcClicked)
        self.clc.clicked.connect(self.clcClicked)
        self.info.clicked.connect(self.infoClicked)
        self.encoder.clicked.connect(self.encoderClicked)
        self.log.clicked.connect(self.logClicked)
        self.lrc.clicked.connect(self.lrcClicked)
        
        # Replace UI combobox with our refreshing one
        old_combo = self.ports
        layout = old_combo.parent().layout()
        self.ports.setObjectName("ports")
        self.ports = RefreshingComboBox(old_combo.parent())
        layout.replaceWidget(old_combo, self.ports)
        old_combo.deleteLater()

        # Set default text before refresh
        self.ports.addItem("Select a port…")
        self.ports.setCurrentIndex(0)

        self.ports.currentTextChanged.connect(self.on_port_selected)

    # Setup recursive function that will retry until data is available
    def try_read_motor_info(self, retry_count=0):
        if retry_count >= 10:  # Limit retries to avoid infinite loop
            return
            
        if self.serial_conn and self.serial_conn.in_waiting > 0:
            self.readMotorInfo()
        else:
            # No data yet, resend request and try again
            self.serial_conn.write(b"i\n")
            QtCore.QTimer.singleShot(200, lambda: self.try_read_motor_info(retry_count + 1))


    def on_port_selected(self, port_name):
        if (not port_name 
            or port_name in ["No ports available", "Select a port…"]):
            # Ignore invalid/default options
            return

        # Close existing connection if open
        if self.serial_conn and self.serial_conn.is_open:
            print(f"Closing {self.serial_conn.port}")
            self.serial_conn.close()

        try:
            # Try to open new connection
            self.serial_conn = serial.Serial(port=port_name, baudrate=115200, timeout=1)
            # Send initial request
            self.serial_conn.write(b"i\n")
            
            # First attempt after 200ms
            QtCore.QTimer.singleShot(200, lambda: self.try_read_motor_info(0))
            
            # print(f"Opened {port_name} at 115200 baud")
            self.set_status("green")   
        except serial.SerialException as e:
            # print(f"Failed to open {port_name}: {e}")
            self.serial_conn = None
            self.set_status("red")  

    def readMotorInfo(self):
        try:
            if not self.serial_conn or not self.serial_conn.in_waiting:
                return

            # Read the first line (status code: 0, 1, or 2)
            while True:
                status_line = self.serial_conn.readline().decode().strip()
                if not status_line:
                    return
                try:
                    status = int(status_line)
                    break  # Got a valid status, exit loop
                except ValueError:
                    continue  # Not a number, skip this line

            if status == 2:
                # Both calibration + motor char exist
                data_line = self.serial_conn.readline().decode().strip()
                ppr, maxRPM = data_line.split()
                self.ppr.setText(f"{ppr}")
                self.maxRPM.setText(f"{float(maxRPM):.2f}")

            elif status == 1:
                # Only calibration exists
                data_line = self.serial_conn.readline().decode().strip()
                ppr = data_line
                self.ppr.setText(f"{ppr}")
                self.maxRPM.setText("   --   ")

            elif status == 0:
                # Nothing stored
                self.ppr.setText("   --   ")
                self.maxRPM.setText("   --   ")

        except Exception as e:
            print("Motor info read error:", e)

    def set_status(self, status):
        """Update QLabel statusIndicator with a given status string."""
        self.statusIndicator.setProperty("status", status)
        self.statusIndicator.style().unpolish(self.statusIndicator)
        self.statusIndicator.style().polish(self.statusIndicator)
        self.statusIndicator.update()

    def close_other_windows(self, keep=[]):
        """Close all child windows except those in 'keep' list."""
        for name, win in list(self.child_windows.items()):
            if name not in keep and win is not None:
                win.close()
                self.child_windows.pop(name, None)

    def olcClicked(self):
        # Handle the OLC button click event
        if self.serial_conn and self.serial_conn.is_open:
            self.close_other_windows(keep=["info"])
            self.serial_conn.write(b"o\n")
            self.child_windows["olc"] = olc(self.serial_conn, self)  # Pass self as main_window
            self.child_windows["olc"].show()
        else:
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port is open.")

    def clcClicked(self):
        # Handle the CLC button click event
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(b"c\n")
        else:
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port is open.")

    def infoClicked(self):
        # Handle the Info button click event
        QtWidgets.QMessageBox.information(self, "Notification", "You pressed the Info button!")
        pass

    def encoderClicked(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.close_other_windows(keep=["info"])
            self.serial_conn.write(b"e\n")
            self.child_windows["encoder"] = Encoder(self.serial_conn, self)  # Pass self as main_window
            self.child_windows["encoder"].show()
        else:
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port is open.")
        
    def logClicked(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.close_other_windows(keep=["info"])
            self.serial_conn.write(b"s\n")
            self.child_windows["logwindow"] = LogWindow(self.serial_conn, self)  # Pass self as main_window
            self.child_windows["logwindow"].show()
        else:
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port is open.")

    def lrcClicked(self):
        # Handle the LRC button click event
        if self.serial_conn and self.serial_conn.is_open:
            self.close_other_windows(keep=["info"])
            self.serial_conn.write(b"l\n")
            self.child_windows["lrc"] = LRC(self.serial_conn, self)  # Pass serial_conn
            self.child_windows["lrc"].show()
        else:
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port is open.")

class Encoder(QtWidgets.QMainWindow):
    def __init__(self, serial_conn, main_window=None):
        super().__init__(main_window)  # Pass parent for Qt hierarchy
        uic.loadUi("ui/calibration.ui", self)
        self.serial_conn = serial_conn
        self.main_window = main_window  # Store reference to main window

        # Connect buttons
        self.pls.pressed.connect(self.plsPressed)
        self.pls.released.connect(self.stopMotor)
        self.min.pressed.connect(self.minPressed)
        self.min.released.connect(self.stopMotor)
        self.done.clicked.connect(self.doneClicked)

        # 🔹 Timer for polling ESP32
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.requestTicks)
        self.timer.start(200)  # every 200 ms (5 Hz) request

    def safe_write(self, data):
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.write(data)
            else:
                QtWidgets.QMessageBox.critical(self, "Serial Error", "Serial connection lost.")
                self.close()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Serial Error", f"Serial error: {e}")
            self.close()

    def plsPressed(self):
        self.timer.stop()  
        self.safe_write(b"1")
        self.serial_conn.flush()
        self.timer.start(200)

    def minPressed(self):
        self.timer.stop()
        self.safe_write(b"-1")
        self.serial_conn.flush()
        self.timer.start(200)

    def stopMotor(self):
        self.timer.stop()
        self.safe_write(b"0")
        self.serial_conn.flush()
        self.timer.start(200)

    def doneClicked(self):
        self.timer.stop()
        try:
            value = float(self.rotation.text())
            # Validate the input
            if value <= 0:
                QtWidgets.QMessageBox.warning(self, "Invalid Input", "Rotation value must be positive.")
                return
            #QtWidgets.QMessageBox.information(self, "Info", f"Storing PPR = {value} to flash.")
            self.serial_conn.write(f"3 {value}".encode())
            self.serial_conn.flush()
            if self.main_window:
                    QtCore.QTimer.singleShot(200, self.main_window.readMotorInfo)
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            self.close()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Serial Error", f"Serial error: {e}")

    # In Encoder class
    def closeEvent(self, event):
        try:
            self.timer.stop()  # Stop polling timer
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.write(b"4")
                
                # Refresh main window after a short delay to let ESP update
                if self.main_window:
                    QtCore.QTimer.singleShot(200, self.main_window.readMotorInfo)

                # Clear serial buffers
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
        except Exception:
            pass
        event.accept()

    def requestTicks(self):
        self.safe_write(b"2")   # Ask ESP for ticks
        self.readSerial()

    def readSerial(self):
        try:
            if self.serial_conn and self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode("utf-8").strip()
                if line:
                    try:
                        ticks = int(line)
                        self.pulse.display(ticks)  # update LCD
                    except ValueError:
                        pass  # ignore garbage/debug lines
        except Exception as e:
            print("Serial read error:", e)

class ProgressBar(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi("ui/progressbar.ui", self)  
        self.setWindowModality(QtCore.Qt.ApplicationModal)

class LogWindow(QtWidgets.QMainWindow):
    def __init__(self, serial_conn, main_window=None):
        super().__init__(main_window)
        uic.loadUi("ui/linlog.ui", self)
        self.serial_conn = serial_conn
        self.main_window = main_window  # Store reference to main window

        # Create chart
        self.chart = QtChart.QChart()
        self.series = QtChart.QLineSeries()
        self.series.setPointsVisible(True)  # Show actual data points
        self.chart.addSeries(self.series)
        self.series.setName("Speed (RPM)")
        self.chart.createDefaultAxes()
        self.chart.setTitle("Motor Characteristic Curve")

        # Chart view
        self.chartView = QtChart.QChartView(self.chart)
        self.chartView.setRenderHint(QtGui.QPainter.Antialiasing)

        # Add chartView into graphFrame's existing grid layout
        layout = self.graphFrame.layout()  # get the QGridLayout already set in QtDesigner
        layout.addWidget(self.chartView, 0, 0, 2, 1)  # row=0, col=0, rowspan=2, colspan=1

        # Enable hover events for tooltips
        self.series.hovered.connect(self.on_hover)

        # Tooltip label
        self.tooltip = QtWidgets.QLabel(self.chartView)
        self.tooltip.setStyleSheet("background-color: white; border: 1px solid black; padding: 2px;")
        self.tooltip.hide()

        # Generate new chart then plot when done
        self.newGraph.clicked.connect(self.newGraphClicked)
        self.popup.clicked.connect(self.popupClicked)

    # Setup recursive function that will retry until data is available
    def try_read_motor_char(self, retry_count=0):
        if retry_count >= 10:  # Limit retries to avoid infinite loop
            QtWidgets.QMessageBox.information(self, "Log", "No motor characteristic data found.")
            return
            
        if self.serial_conn and self.serial_conn.in_waiting > 0:
            self.pwm, self.speed = self.read_motor_characteristic()
            self.load_and_plot(self.pwm, self.speed)
        else:
            # No data yet, resend request and try again
            self.serial_conn.write(b"2")
            QtCore.QTimer.singleShot(200, lambda: self.try_read_motor_char(retry_count + 1))

    def showEvent(self, event):
        """Called when window is shown"""
        super().showEvent(event)
        self.try_read_motor_char()

    def load_and_plot(self, pwm, speed):
        self.series.clear()

        if pwm and speed:
            for x, y in zip(pwm, speed):
                self.series.append(x, y)

            self.chart.axisX().setTitleText("PWM Value")
            self.chart.axisY().setTitleText("Speed (RPM)")

            # Set axis ranges
            self.chart.axisX().setRange(min(pwm), max(pwm))
            self.chart.axisY().setRange(0, max(speed) * 1.1)  # Add 10% margin
        else:
            QtWidgets.QMessageBox.information(self, "Log", "No motor characteristic file found.")

    def read_motor_characteristic(self):
        data_pwm, data_speed = [], []
        first_line = self.serial_conn.readline().decode("utf-8").strip()
        if first_line == "0":
            return None, None
        elif first_line == "1":
            while True:
                line = self.serial_conn.readline().decode("utf-8").strip()
                if not line:
                    break
                try:
                    pwm, speed = map(float, line.split())
                    data_pwm.append(pwm)
                    data_speed.append(speed)
                except ValueError:
                    break
        return data_pwm, data_speed

    def on_hover(self, point, state):
        """Show tooltip when hovering points"""
        if state:
            self.tooltip.setText(f"PWM: {point.x():.1f} Speed: {point.y():.1f}")
            self.tooltip.adjustSize()

            # Position tooltip near mouse (top left instead of top right)
            cursor_pos = QtGui.QCursor.pos()
            widget_pos = self.chartView.mapFromGlobal(cursor_pos)
            self.tooltip.move(widget_pos.x() - self.tooltip.width() - 10, widget_pos.y() - 20)
            self.tooltip.show()
        else:
            self.tooltip.hide()

    def newGraphClicked(self):
        if self.serial_conn and self.serial_conn.is_open:
            # Tell ESP32 to start logging
            self.serial_conn.write(b"1")

            # Show modal progress bar
            self.progressDialog = ProgressBar(self)
            self.progressDialog.setModal(True)
            self.progressDialog.show()

            # Start a timer to poll serial port
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.readProgress)
            self.timer.start(100)  # check every 100 ms
        else:
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port is open.")

    def readProgress(self):
        try:
            if self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode("utf-8").strip()
                if not line:
                    return

                try:
                    progress = float(line)
                    #print(f"Progress: {progress}%")
                    self.progressDialog.progressBar.setValue(int(progress))
                    self.progressDialog.progressBar.setFormat(f"{progress:.2f}%") 
                    
                    # if finished
                    if progress >= 100.0:
                        self.timer.stop()
                        self.progressDialog.progressBar.setValue(100)
                        self.progressDialog.progressBar.setFormat("100%")
                        self.timer.stop()
                    
                        # Delay 1 second before closing
                        QtCore.QTimer.singleShot(1000, lambda: (
                            self.try_read_motor_char(),  # Refresh graph
                            self.progressDialog.accept(),
                        ))
                except ValueError:
                    # ignore garbage/extra lines
                    pass

        except Exception as e:
            print("Serial error:", e)

    def popupClicked(self):
        if hasattr(self, 'pwm') and hasattr(self, 'speed') and self.pwm and self.speed:
            plt.figure(figsize=(10, 6))
            plt.plot(self.pwm, self.speed, 'o-', color='blue', linewidth=2)
            plt.grid(True)
            plt.xlabel('PWM Value')
            plt.ylabel('Speed (RPM)')
            plt.title('Motor Characteristic Curve')
            plt.tight_layout()
            plt.show(block=False)  # Non-blocking show - window persists when LogWindow closes
        else:
            QtWidgets.QMessageBox.information(self, "Log", "No data available to plot.")

    def closeEvent(self, event):
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.write(b"4")
                
                # Refresh main window after a short delay to let ESP update
                if self.main_window:
                    QtCore.QTimer.singleShot(200, self.main_window.readMotorInfo)

                # Clear serial buffers
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
        except Exception:
            pass
        event.accept()

class LRC(QtWidgets.QMainWindow):
    def __init__(self, serial_conn, main_window=None):
        super().__init__(main_window)
        uic.loadUi("ui/lrc.ui", self)
        self.serial_conn = serial_conn

        #read existing linear info
        QtCore.QTimer.singleShot(200, self.readLinearInfo)  # wait 200 ms then read

        # Connect buttons
        self.done.clicked.connect(self.doneClicked)

    def readLinearInfo(self, retry_count=0):
        if retry_count >= 10:  # Limit retries to avoid infinite loop
            return
        
        try:
            if not self.serial_conn or not self.serial_conn.in_waiting:
                self.serial_conn.write(b"2")
                QtCore.QTimer.singleShot(200, lambda: self.readLinearInfo(retry_count + 1))

            else:
                # Read the first line (status code: 0 or 1)
                while True:
                    status_line = self.serial_conn.readline().decode().strip()
                    if not status_line:
                        return
                    try:
                        status = int(status_line)
                        break  # Got a valid status, exit loop
                    except ValueError:
                        continue  # Not a number, skip this line

                if status == 1:
                    # Both calibration + motor char exist
                    data_line = self.serial_conn.readline().decode().strip()
                    minLinPWM, maxLinPWM, minLinRPM, maxLinRPM = data_line.split()
                    self.minLinDisp.setText(f"{minLinPWM}")
                    self.maxLinDisp.setText(f"{maxLinPWM}")
                    self.MinRPMDisp.setText(f"{float(minLinRPM):.2f}")
                    self.MaxRPMDisp.setText(f"{float(maxLinRPM):.2f}")

                elif status == 0:
                    # Nothing stored
                    self.minLinDisp.setText("--")
                    self.maxLinDisp.setText("--")
                    self.MinRPMDisp.setText("--")
                    self.MaxRPMDisp.setText("--")

        except Exception as e:
            print("Motor info read error:", e)

    def safe_write(self, data):
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.write(data)
            else:
                QtWidgets.QMessageBox.critical(self, "Serial Error", "Serial connection lost.")
                self.close()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Serial Error", f"Serial error: {e}")
            self.close()

    def doneClicked(self):
        try:
            valueMin = int(self.minLinPWM.text())
            valueMax = int(self.maxLinPWM.text())
            valueRPMMin = float(self.minLinRPM.text())
            valueRPMMax = float(self.maxLinRPM.text())
            # Check for negative values
            if valueMin < 0 or valueMax < 0 or valueRPMMin < 0.0 or valueRPMMax < 0.0:
                QtWidgets.QMessageBox.warning(self, "Invalid Input", "Negative values are not allowed.")
                return
            # Check if min > max
            if valueMin >= valueMax:
                QtWidgets.QMessageBox.warning(self, "Invalid Input", "Minimum PWM must be less than maximum PWM.")
                return

            if valueRPMMin >= valueRPMMax:
                QtWidgets.QMessageBox.warning(self, "Invalid Input", "Minimum RPM must be less than maximum RPM.")
                return
            self.safe_write(f"3 {valueMin} {valueMax} {valueRPMMin} {valueRPMMax}".encode())  # Save LRC
            self.serial_conn.flush()
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            self.readLinearInfo()  # Refresh display
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Serial Error", f"Serial error: {e}")

    def closeEvent(self, event):
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.safe_write(b"4")  # Stop LRC reading on close
                # Clear serial buffers
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
        except Exception:
            pass
        event.accept()

class olc(QtWidgets.QMainWindow):
    def __init__(self, serial_conn, main_window=None):
        super().__init__(main_window)
        uic.loadUi("ui/olc.ui", self)
        self.serial_conn = serial_conn
        self.main_window = main_window  # Store reference to main window
        QtCore.QTimer.singleShot(200, self.try_read_motor_characteristic)  # wait 200 ms then read

        # Connect buttons
        self.start.clicked.connect(self.startClicked)
        self.analyze.clicked.connect(self.analyzeClicked)
        self.popup.clicked.connect(self.popupClicked)

        # Create real-time chart for transient response
        self.chart = QtChart.QChart()
        self.speedSeries = QtChart.QLineSeries()  # Actual speed
        self.targetSeries = QtChart.QLineSeries()  # Target speed
        
        # Set series names for legend
        self.speedSeries.setName("Actual Speed")
        self.targetSeries.setName("Target Speed")
        
        # Add series to chart
        self.chart.addSeries(self.speedSeries)
        self.chart.addSeries(self.targetSeries)
        self.speedSeries.setColor(QtGui.QColor("blue"))
        self.targetSeries.setColor(QtGui.QColor("red"))
        #self.speedSeries.setUseOpenGL(True)  # Enable OpenGL for better performance
        #self.targetSeries.setUseOpenGL(True)  # Enable OpenGL for better performance
        self.chart.legend().setVisible(True)
        self.chart.legend().setAlignment(QtCore.Qt.AlignTop)
        self.chart.legend().setFont(QtGui.QFont("Arial", 10))
        self.speedSeries.setName("Speed (RPM)")
        self.targetSeries.setName("Target (RPM)")
        
        # Create axes
        self.chart.createDefaultAxes()
        self.chart.axisX().setTitleText("Time (ms)")
        self.chart.axisY().setTitleText("Speed (RPM)")
        self.chart.setTitle("Motor Transient Response")

        # Setup chart view
        self.chartView = QtChart.QChartView(self.chart)
        self.chartView.setRenderHint(QtGui.QPainter.Antialiasing)
        
        # Add chartView into olcgraph 
        layout = self.olcgraph.layout()
        if not layout:
            layout = QtWidgets.QGridLayout(self.olcgraph)
        layout.addWidget(self.chartView, 0, 0, 2, 1)  # row=0, col=0, rowspan=2, colspan=1

        # Enable hover events for tooltips
        self.speedSeries.hovered.connect(self.on_hover)
        self.targetSeries.hovered.connect(self.on_hover)

        # Tooltip label
        self.tooltip = QtWidgets.QLabel(self.chartView)
        self.tooltip.setStyleSheet("background-color: white; border: 1px solid black; padding: 2px;")
        self.tooltip.hide()

    def on_hover(self, point, state):
        """Show tooltip when hovering points"""
        if state:
            self.tooltip.setText(f"Time: {point.x():.1f} ms\nSpeed: {point.y():.1f} RPM")
            self.tooltip.adjustSize()

            # Position tooltip near mouse (top left instead of top right)
            cursor_pos = QtGui.QCursor.pos()
            widget_pos = self.chartView.mapFromGlobal(cursor_pos)
            self.tooltip.move(widget_pos.x() - self.tooltip.width() - 10, widget_pos.y() - 20)
            self.tooltip.show()
        else:
            self.tooltip.hide()

    def try_read_motor_characteristic(self, retry_count=0):
        if retry_count >= 10:  # Limit retries to avoid infinite loop
            return
            
        if self.serial_conn and self.serial_conn.in_waiting > 0:
            self.readMotorData()
        else:
            # No data yet, resend request and try again
            self.serial_conn.write(b"2")
            QtCore.QTimer.singleShot(200, lambda: self.try_read_motor_characteristic(retry_count + 1))

    def readMotorData(self):
        try:
            if not self.serial_conn or not self.serial_conn.in_waiting:
                return

            # Read the first line (status code: 0, 1, or 2)
            while True:
                status_line = self.serial_conn.readline().decode().strip()
                if not status_line:
                    return
                try:
                    status = int(status_line)
                    break  # Got a valid status, exit loop
                except ValueError:
                    continue  # Not a number, skip this line

            if status == 2:
                # Both calibration + motor char exist
                data_line = self.serial_conn.readline().decode().strip()
                min_rpm_str, max_rpm_str, max_rpm_val_str = data_line.split()
                
                # Convert to float when storing
                self.minLinRPM = float(min_rpm_str)
                self.maxLinRPM = float(max_rpm_str)
                self.maxRPM = float(max_rpm_val_str)
                
                # Display values
                self.MinLinRPMDisp.setText(f"{self.minLinRPM:.2f}")
                self.MaxLinRPMDisp.setText(f"{self.maxLinRPM:.2f}")
                self.MaxRPMDisp.setText(f"{self.maxRPM:.2f}")

            elif status == 1:
                # Only calibration exists
                data_line = self.serial_conn.readline().decode().strip()
                self.maxRPM = float(data_line)
                self.MinLinRPMDisp.setText("--")
                self.MaxLinRPMDisp.setText("--")
                self.MaxRPMDisp.setText(f"{float(self.maxRPM):.2f}")

            elif status == 0:
                # Nothing stored
                self.MinLinRPMDisp.setText("--")
                self.MaxLinRPMDisp.setText("--")
                self.MaxRPMDisp.setText("--")
        except Exception as e:
            print("Serial read error:", e)

    def startClicked(self):
        if self.serial_conn and self.serial_conn.is_open:
            targetRPM = float(self.targetRPM.text())
            # Validate input
            if targetRPM <= 0:
                QtWidgets.QMessageBox.warning(self, "Invalid Input", "Target RPM must be positive.")
                return
            # Check if target RPM is within valid ranges
            try:
                # Check which data is available
                print(f"MinLinRPM: {self.minLinRPM}, MaxLinRPM: {self.maxLinRPM}, MaxRPM: {self.maxRPM}")
                has_linear_range = hasattr(self, 'minLinRPM') and hasattr(self, 'maxLinRPM')
                has_max_rpm = hasattr(self, 'maxRPM')
                
                if has_linear_range and has_max_rpm:
                    # Both linear range and max RPM are available
                    min_lin_rpm = float(self.minLinRPM)
                    max_lin_rpm = float(self.maxLinRPM)
                    max_rpm = float(self.maxRPM)
                    
                    # Check if target is between min and max linear RPM OR equals max RPM
                    is_in_linear_range = min_lin_rpm <= targetRPM <= max_lin_rpm
                    is_at_max = abs(targetRPM - max_rpm) < 0.01  # Small epsilon for float comparison
                    
                    if not (is_in_linear_range or is_at_max):
                        QtWidgets.QMessageBox.warning(
                            self, 
                            "Invalid Input", 
                            f"Target RPM must be either between {min_lin_rpm:.2f} and {max_lin_rpm:.2f}, or equal to {max_rpm:.2f}."
                        )
                        return
                    
                elif has_max_rpm:
                    # Only max RPM is available
                    max_rpm = float(self.maxRPM)
                    
                    if targetRPM > max_rpm:
                        QtWidgets.QMessageBox.warning(
                            self, 
                            "Invalid Input", 
                            f"Target RPM cannot exceed maximum RPM ({max_rpm:.2f})."
                        )
                        return
                    
                else:
                    # No motor data available
                    QtWidgets.QMessageBox.warning(
                        self, 
                        "Missing Data", 
                        "Motor characterization data not available. Run analysis first."
                    )
                    return
            except (ValueError, AttributeError):
                # Handle case where values aren't set yet or are invalid
                QtWidgets.QMessageBox.warning(
                    self, 
                    "Missing Data", 
                    "Motor characterization file not available. Run analysis first."
                )
                return
                
            # If we get here, targetRPM is valid, send it to ESP32
            self.readTransientResponse(targetRPM)
        else:
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port is open.")

    def safe_write(self, data):
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.write(data)
            else:
                QtWidgets.QMessageBox.critical(self, "Serial Error", "Serial connection lost.")
                self.close()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Serial Error", f"Serial error: {e}")
            self.close()

    def readTransientResponse(self, targetRPM):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.reset_input_buffer()

            #Prepare arrays
            self.time_data = []
            self.rpm_data = []
            self.target_data = []

            self.speedSeries.clear()
            self.targetSeries.clear()

            self.safe_write(f"1 {targetRPM}".encode())
            self.serial_conn.flush()

            # Start timer to poll for data
            self.responseTimer = QtCore.QTimer(self)
            self.responseTimer.timeout.connect(self.updateTransientPlot)
            self.responseTimer.start(10)  # Poll every 10ms for fast updates

    def updateTransientPlot(self):
        try:
            if not self.serial_conn or not self.serial_conn.is_open:
                self.responseTimer.stop()
                return
                
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode("utf-8").strip()
                
                # Check if data collection is complete
                if line == "DONE":
                    self.responseTimer.stop()
                    return

                 # Skip the header line
                if line == "time_ms,rpm,targetrpm":
                    print("Received header, starting data collection...")
                    return
                    
                try:
                    # Parse the CSV format: timestamp,actual_speed,target_speed
                    parts = line.split(',')
                    if len(parts) == 3:
                        timestamp = int(parts[0])  # milliseconds
                        actual_speed = float(parts[1])
                        target_speed = float(parts[2])
                        
                        # Store data
                        self.time_data.append(timestamp)
                        self.rpm_data.append(actual_speed)
                        self.target_data.append(target_speed)
                        
                        # Add to chart series
                        self.speedSeries.append(timestamp, actual_speed)
                        self.targetSeries.append(timestamp, target_speed)
                        
                        # Update chart axes to fit data
                        self.chart.axisX().setRange(0, max(self.time_data) + 100)
                        
                        # Calculate y-axis range to fit data with some margin
                        max_y = max(max(self.rpm_data, default=0), target_speed) * 1.1
                        min_y = min(min(self.rpm_data, default=0), 0) * 0.9
                        self.chart.axisY().setRange(min_y, max_y)
                        
                except (ValueError, IndexError) as e:
                    print(f"Error parsing data: {e}")
                    # Skip invalid lines
                    pass
                    
        except Exception as e:
            print(f"Error in updateTransientPlot: {e}")
            self.responseTimer.stop()
        
    def popupClicked(self):
        if hasattr(self, 'time_data') and hasattr(self, 'rpm_data') and self.time_data and self.rpm_data:
            plt.figure(figsize=(10, 6))
            plt.plot(self.time_data, self.rpm_data, 'o-', color='blue', linewidth=2, label='Actual Speed')
            plt.plot(self.time_data, self.target_data, 'r--', label='Target Speed')
            plt.grid(True)
            plt.xlabel('Time (ms)')
            plt.ylabel('Speed (RPM)')
            plt.legend()
            plt.title('Motor Speed Response')
            plt.show()

    def analyzeClicked(self):
        if not self.time_data or not self.rpm_data:
            return
            
        target = self.target_data[0] if self.target_data else 0
        max_speed = max(self.rpm_data)
        
        # Find rise time (10% to 90% of target)
        start_time = self.time_data[0]
        threshold_10 = 0.1 * target
        threshold_28_3 = 0.283 * target
        threshold_63_2 = 0.632 * target
        threshold_90 = 0.9 * target

        rise_start = None
        time_28_3 = None
        time_63_2 = None
        rise_end = None

        for i, speed in enumerate(self.rpm_data):
            if rise_start is None and speed >= threshold_10:
                rise_start = self.time_data[i]
            if time_28_3 is None and speed >= threshold_28_3:
                time_28_3 = self.time_data[i]
            if time_63_2 is None and speed >= threshold_63_2:
                time_63_2 = self.time_data[i]
            if rise_start is not None and speed >= threshold_90:
                rise_end = self.time_data[i]
                break
                    
        rise_time = (rise_end - rise_start) if rise_start and rise_end else 0
        t_28_3 = (time_28_3 - start_time) if start_time and time_28_3 else 0
        t_63_2 = (time_63_2 - start_time) if start_time and time_63_2 else 0

        #Calculate final value
        fv = self.rpm_data[-1]

        #Find settling time 2% oscillation band
        settling_time = 0
        if rise_end:
            upper_bound = fv * 1.02
            lower_bound = fv * 0.98
            for i in range(len(self.rpm_data)-1, -1, -1):
                if not (lower_bound <= self.rpm_data[i] <= upper_bound):
                    settling_time = self.time_data[i+1] - start_time if (i+1) < len(self.time_data) else 0
                    break

        #Display values
        self.Tr.setText(f"{rise_time} ms" if rise_time else "--")
        self.t28.setText(f"{t_28_3} ms" if t_28_3 else "--")
        self.t63.setText(f"{t_63_2} ms" if t_63_2 else "--")
        self.Ts.setText(f"{settling_time} ms" if settling_time else "--")
        self.fv.setText(f"{fv:.2f} RPM" if fv else "--")
        self.tau.setText(f"{t_63_2:.2f} ms" if t_63_2 else "--")


    def doneClicked(self):
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.write(b"4")  # Stop OLC reading on close
                
                # Refresh main window after a short delay to let ESP update
                if self.main_window:
                    QtCore.QTimer.singleShot(200, self.main_window.readMotorInfo)

                # Clear serial buffers
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
            self.close()
        except Exception:
            pass

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
