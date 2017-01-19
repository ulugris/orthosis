#!/usr/bin/python
# coding: latin-1

import sip
sip.setapi('QVariant', 2)

import numpy as np
import sys, struct

from functools import partial
from guiqwt.builder import make

# Try Qt5, fall back to Qt4 if Qt5 not available
try:
	from PyQt5.QtGui import QColor, QPen
	from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QFrame
	from PyQt5.QtCore import QSettings, QPoint, QSize, Qt, QTimer
	from PyQt5.QtNetwork import QUdpSocket, QHostAddress
	from PyQt5.uic import loadUi
except ImportError:
	from PyQt4.QtGui import QColor, QPen
	from PyQt4.QtGui import QApplication, QMainWindow, QMessageBox, QFrame
	from PyQt4.QtCore import QSettings, QPoint, QSize, Qt, QTimer
	from PyQt4.QtNetwork import QUdpSocket, QHostAddress
	from PyQt4.uic import loadUi


class RTPlot(QMainWindow):
	LOCALIP = "192.168.137.1"

	# Local interfaces for command and plotting sockets
	CMDADDR = (QHostAddress(LOCALIP), 0)
	PLTADDR = (QHostAddress(LOCALIP), 8889)

	# Orthosis UDP server address
	UDPADDR = (QHostAddress.Broadcast, 8888)

	# True if knobs are already connected
	KNCONN = False

	# RTPlot class constructor
	def __init__(self, parent = None):
		super(RTPlot, self).__init__(parent)
		loadUi('Orthosis.ui', self)

		plotLen = 15                               # Plot length (seconds)
		self.fr = 25.0                             # Expected frame rate (Hz)
		self.nFrames = int(round(self.fr*plotLen)) # Number of frames

		# Initialize data arrays for plots
		self.frame = 0
		self.x = range(self.nFrames)
		self.y = [np.zeros(self.nFrames) for i in range(8)]

		# Store default text color for knob labels
		defColor = vars(self)['timelabel'].palette().text().color()
		self.defColor = (defColor.red(), defColor.green(), defColor.blue())
		self.defStyle = "QLabel {color: rgb(%i, %i, %i)}" % self.defColor

		# Read GUI settings from INI file
		self.st = QSettings('Orthosis.ini', QSettings.IniFormat)
		self.resize(self.st.value('size', QSize(800, 450)))
		self.move(self.st.value('pos', QPoint(50, 50)))

		# Number of parameters per leg
		self.npr = 10

		# Minimum and maximum knob values (in actual units)
		self.mn = [  0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0,   0, -25.0, -25.0]
		self.mx = [100.0, +1.0, +1.0, 2.0, 1.0, 0.5, 0.5, 100,  25.0,  25.0]

		# Knob parameter scaling factors (from 0-100 value to actual units)
		self.ml = [0.01*(self.mx[i] - self.mn[i]) for i in range(self.npr)]

		# Store knobs and labels into lists for easier manipulation
		self.knobs = []
		self.label = []
		for ch, leg in zip(range(2), ['r', 'l']):
			for kn in range(self.npr):
				self.knobs.append(vars(self)[leg + "k" + str(kn + 1)])
				self.label.append(vars(self)[leg + "d" + str(kn + 1)])

		# Create UDP sockets
		self.sc = QUdpSocket() # Socket for sending commands
		self.sp = QUdpSocket() # Socket for receiving plot data

		# Bind UDP sockets to their interfaces and ports
		self.sc.bind(self.CMDADDR[0], self.CMDADDR[1])
		self.sp.bind(self.PLTADDR[0], self.PLTADDR[1])

		# Qt signal and slot connections
		self.powerButton.clicked.connect(self.power)
		self.startButton.clicked.connect(self.trigger)
		self.sp.readyRead.connect(self.update)

		# Initialize plot and show GUI
		self.initPlot()
		self.show()

		# Try co connect to Beaglebone
		QTimer.singleShot(10, lambda: self.connectGUI())

	# Save GUI settings before closing window
	def closeEvent(self, event):
		self.st.setValue('size', self.size())
		self.st.setValue('pos', self.pos())

		event.accept()
		self.st.sync()
		sys.exit()

	# Connect to Beaglebone and apply settings to GUI
	def connectGUI(self):
		self.sendCommand(b"Connect")

		if self.checkCommand(5000):
			# Get current system status
			self.sc.waitForReadyRead(100)
			status = self.sc.readDatagram(1)[0]
			self.initButtons(ord(status))

			# Get current system parameters
			self.sc.waitForReadyRead(100)
			par = list(struct.unpack("20d", self.sc.readDatagram(1024)[0]))

			# Set knob values and connect callbacks
			for kn in range(self.npr):
				mul = self.ml[kn]
				off = self.mn[kn]
				for ch in range(2):
					# Knob index in list
					i = self.npr*ch + kn

					# Convert knob value to QDial units (0-100)
					kval = (par[i] - off)/mul

					# Set knob value and update label
					self.knobs[i].setValue(kval)
					self.knobs[i].setEnabled(True)
					self.label[i].setText("%1.2f" % par[i])

					# Connect callback to knob
					if self.KNCONN == False:
						callback = partial(self.knobMove, ch, kn, mul, off)
						self.knobs[i].valueChanged.connect(callback)

			self.KNCONN = True
			self.connected = True

	# Initialize plots
	def initPlot(self):
		self.rMtr = make.curve([], [], color='#245745', linewidth=3.0)
		self.lMtr = make.curve([], [], color='#752929', linewidth=3.0)
		self.rAcc = make.curve([], [], color='#777777', linewidth=1.0)
		self.lAcc = make.curve([], [], color='#777777', linewidth=1.0)
		self.rAng = make.curve([], [], color='#00dd00', linewidth=2.0)
		self.lAng = make.curve([], [], color='#ff0000', linewidth=2.0)

		self.plots = [self.RPlot, self.LPlot]
		self.curvs = [self.rMtr, self.lMtr, self.rAcc, self.lAcc, self.rAng, self.lAng]
		self.crvmp = [3, 4, 6, 7, 1, 2]

		majPen = QPen(QColor('#646464'), 1.0, Qt.DotLine)
		minPen = QPen(QColor('#323232'), 1.0, Qt.DotLine)

		for plt in self.plots:
			plt.set_plot_limits(0, self.nFrames-1, -0.1, 70.1)
			plt.canvas().setFrameShape(QFrame.StyledPanel)
			plt.setCanvasBackground(Qt.black)
			plt.plotLayout().setCanvasMargin(-1)
			plt.set_antialiasing(True)
			try:
				plt.grid.setMajorPen(majPen)
				plt.grid.setMinorPen(minPen)
			except AttributeError:
				plt.grid.setMajPen(majPen)
				plt.grid.setMinPen(minPen)

			for i in range(4):
				plt.enableAxis(i, False)

		corresp = [0, 1, 0, 1, 0, 1, 0, 1]
		for plt, crv in zip(corresp, self.curvs):
			self.plots[plt].add_item(crv)

	# Convert LiPo voltage to charge percentage
	def chrg(self, V):
		return 410.974*V**3 - 5189.38*V**2 + 21935.0*V - 30935.5

	# Update plot
	def update(self):
		line = self.sp.readDatagram(1024)[0]
		data = list(struct.unpack('8d', line))

		m = int(data[0]/60)
		if m < 1:
			time = 'Time: %1.1f s' % (data[0])
		else:
			time = 'Time: %1.0f min %1.1f s' % (m, data[0] - 60*m)
		self.timelabel.setText(time)

		nf = min(self.frame, self.nFrames)
		for i in range(len(self.crvmp)):
			self.y[i] = np.hstack([data[self.crvmp[i]], np.delete(self.y[i], -1)])
			self.curvs[i].set_data(self.x[0:nf], self.y[i][0:nf])

		for i in range(2):
			self.plots[i].replot()

		if data[5]:
			data[5] += 0.01
			self.LiPo.setText('LiPo: %0.2f V (~%i%%)' % (5.0*data[5], self.chrg(data[5])))

		self.frame += 1

	def knobMove(self, ch, kn, mul, off, val):
		# Convert value to actual units
		sval = off + mul*val

		# Pack command into a byte array and send
		self.sendCommand(struct.pack('3d', ch, kn, sval))

		# Check if command has been registered
		styleSheet = self.defStyle
		check = self.checkCommand(1000)

		# Change label only if answer is received
		if check:
			self.label[self.npr*ch + kn].setText("%1.2f" % sval)

			# Turn labels red if trajectory is not feasible
			if check == -1:
				styleSheet = "QLabel {color: red}"

		for i in range(4):
			self.label[self.npr*ch + i].setStyleSheet(styleSheet)

	# Callback for power button
	def power(self):
		if self.powerButton.isChecked():
			if self.connected:
				self.sendCommand(b"On")

				if self.checkCommand(5000):
					self.initButtons(1)

			else:
				self.connectGUI()

		else:
			# Warn user about shutting down motors
			msgBox = QMessageBox(self)
			msgBox.setIcon(QMessageBox.Warning)
			msgBox.setWindowTitle("Shutting down motors")
			msgBox.setText("Warning:\n\n  Motors will shut down, are you sure?")
			msgBox.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
			msgBox.setDefaultButton(QMessageBox.No)

			if msgBox.exec_() == QMessageBox.Yes:
				self.sendCommand(b"Off")

				if self.checkCommand(2500):
					self.initButtons(0)
			else:
				self.powerButton.setChecked(True)

	# Callback for Start/Stop button
	def trigger(self):
		if self.startButton.isChecked():
			self.clearSocket(self.sp)
			self.sendCommand(b"Start")

			if self.checkCommand(2500):
				self.initButtons(3)
				self.frame = 0

		else:
			self.sendCommand(b"Stop")

			if self.checkCommand(2500):
				self.initButtons(1)

	# Configure buttons according to current system status
	# -1: disconnected; 0: disabled; 1: enabled; 3: running
	def initButtons(self, status):
		if status < 1:
			self.powerButton.setChecked(False)
			if status == -1:
				self.powerButton.setText("Connect")
			else:
				self.powerButton.setText("Enable")
			self.powerButton.setStyleSheet('QPushButton {color: black}')

			self.startButton.setChecked(False)
			self.startButton.setEnabled(False)
			self.startButton.setText("Start")
		else:
			self.powerButton.setChecked(True)
			self.powerButton.setText("Disable")
			self.powerButton.setStyleSheet("QPushButton {font: bold; color: red}")

			if status == 1:
				self.startButton.setChecked(False)
				self.startButton.setEnabled(True)
				self.startButton.setText("Start")
			if status == 3:
				self.startButton.setChecked(True)
				self.startButton.setEnabled(True)
				self.startButton.setText("Stop")

	def sendCommand(self, command):
		self.clearSocket(self.sc)
		self.sc.writeDatagram(command, self.UDPADDR[0], self.UDPADDR[1])

	def checkCommand(self, timeout = 100, ans = b"Ok"):
		self.sc.waitForReadyRead(timeout)

		if self.sc.hasPendingDatagrams():
			rcv = self.sc.readDatagram(1024)[0]
		else:
			rcv = []

		if rcv == ans:
			return 1
		elif rcv == []:
			self.initButtons(-1)
			for kn in self.knobs:
				kn.setEnabled(False)

			self.connected = False

			QMessageBox.critical(self, "Error", "Not connected to server")

			return 0
		else:
			return -1

	def clearSocket(self, socket):
		while socket.hasPendingDatagrams():
			print("Discarding packet")
			socket.readDatagram(1024)
			socket.waitForReadyRead(100)

if __name__ == '__main__':
	app = QApplication(sys.argv)
	app.setQuitOnLastWindowClosed(True)
	window = RTPlot()

	sys.exit(app.exec_())
