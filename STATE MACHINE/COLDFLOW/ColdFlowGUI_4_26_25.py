from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget, QPushButton, QVBoxLayout, QLabel
import pyqtgraph as pg
from PyQt5.QtCore import QTimer, Qt
import sys
from threading import Thread
from serial import Serial
from collections import deque
import csv
from os.path import isfile
import time
import random

# Settings
data_len = 1500
num_plots = 6
BUFFER_SIZE = 100
write_buffer = []

# Create deques for all data streams
deque_list = [deque(maxlen=data_len) for _ in range(18)]
(
    x,
    PT_FUEL_UP,
    PT_FUEL_DOWN,
    PT_FUEL_THRESH_UP,
    PT_FUEL_THRESH_DOWN,
    PT_LOX_UP,
    PT_LOX_DOWN,
    PT_LOX_THRESH_UP,
    PT_LOX_THRESH_DOWN,
    PT_I,
    PT_FUEL_UP2,
    INSTABILITY,
    DP_LOX,
    DP_FUEL,
    DP,
    FUEL_STATE,
    LOX_STATE,
    dummy
) = deque_list

plot_titles = [
    "Fuel Pressures (Tank, Downstream, Thresholds)",
    "LOX Pressures (Tank, Downstream, Thresholds)",
    "Injector Pressure",
    "Upstream Pressure",
    "Instabilities and Derivatives",
    "Fuel and LOX States"
]
button_names = ['  Idle  ', ' Armed ', 'Pressed', '   Pressed2   ', ' Fills ', 'Fire', ' Abort ']

# Serial setup
USE_FAKE_SERIAL = False  # should be false wehn ACTUALLY testing (true is just for random data)

if USE_FAKE_SERIAL:
    class FakeSerial:
        def readline(self):
            # Simulate 10 random sensor readings (float), plus system states (COM, DAQ, etc.)
            fake_values = [round(random.uniform(10, 100), 2) for _ in range(16)]
            fake_flags = [
                str(random.randint(0, 6)),  # COM_S
                "1",  # DAQ_S
                "1",  # FLIGHT_S
                "False", "False", "False", "False",  # AUTO_ABORT, ETH_VENT, OX_VENT, OX_COMPLETE
                "0"  # FLIGHT_Q_LENGTH
            ]
            line = "START," + ",".join(map(str, fake_values + fake_flags)) + ",END\n"
            return line.encode('utf-8')
        
        def write(self, data):
            # Print out what the GUI tries to send (for buttons etc.)
            print(f"[FakeSerial] Command sent: {data.decode('utf-8')}")
    
    esp32 = FakeSerial()

else:
    from serial import Serial
    # for mac port_num = "/dev/cu.usbserial-0001"
    port_num = "COM11"  # CHECK YOUR PORT !!!
    esp32 = Serial(port=port_num, baudrate=115200)
    # !!! IF NO NUMBERS PRINTED ON UR TERMINAL => PRESS "EN" ON THE ESP !!!


# CSV setup
file_base = f"ColdFlowLE3_{time.strftime('%Y-%m-%d', time.gmtime())}"
file_ext = ".csv"
test_num = 1
while isfile(file_base + f"_test{test_num}" + file_ext):
    test_num += 1
filename = file_base + f"_test{test_num}" + file_ext

# Serial collection thread
def collection():
    with open(filename, "a", newline='') as f:
        writer = csv.writer(f)
        while True:
            data = esp32.readline()
            try:
                decoded_line = data.decode("utf-8").strip()
                if decoded_line.startswith("START") and decoded_line.endswith("END"):
                    trimmed = decoded_line[6:-4]
                    values = trimmed.split(",")
                    if len(values) >= 16:
                        x.append(time.time())
                        PT_FUEL_UP.append(float(values[0]))
                        PT_FUEL_DOWN.append(float(values[1]))
                        PT_FUEL_THRESH_UP.append(float(values[2]))
                        PT_FUEL_THRESH_DOWN.append(float(values[3]))
                        PT_LOX_UP.append(float(values[4]))
                        PT_LOX_DOWN.append(float(values[5]))
                        PT_LOX_THRESH_UP.append(float(values[6]))
                        PT_LOX_THRESH_DOWN.append(float(values[7]))
                        PT_I.append(float(values[8]))
                        PT_FUEL_UP2.append(float(values[9]))
                        INSTABILITY.append(float(values[10]))
                        DP_LOX.append(float(values[11]))
                        DP_FUEL.append(float(values[12]))
                        DP.append(float(values[13]))
                        FUEL_STATE.append(float(values[14]))
                        LOX_STATE.append(float(values[15]))

                        write_buffer.append(values)

                if len(write_buffer) >= BUFFER_SIZE:
                    writer.writerows(write_buffer)
                    write_buffer.clear()

            except Exception as e:
                print(f"Error: {e}")
                continue

# Main GUI class
class LivePlotter(QMainWindow):
    def __init__(self):
        super(LivePlotter, self).__init__()

        self.centralWidget = QWidget()
        self.setCentralWidget(self.centralWidget)
        self.layout = QGridLayout(self.centralWidget)

        # Colors for plots
        self.colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 165, 0)
        ]

        self.plotDataItems = [[] for _ in range(num_plots)]
        self.graphWidgets = []

        for i, title in enumerate(plot_titles):
            graphWidget = pg.PlotWidget(title=title)
            graphWidget.setFixedHeight(650)
            graphWidget.showGrid(x=True, y=True)
            self.layout.addWidget(graphWidget, i // 3 + 1, i % 3)
            self.graphWidgets.append(graphWidget)

            # Add multiple plot lines depending on graph
            lines_needed = [4, 4, 1, 1, 4, 2][i]
            for j in range(lines_needed):
                pen = pg.mkPen(color=self.colors[j % len(self.colors)], width=2)
                plot_item = graphWidget.plot([], [], pen=pen)
                self.plotDataItems[i].append(plot_item)

        # Buttons
        self.buttonLayout = QVBoxLayout()
        self.buttons = []
        for i, name in enumerate(button_names):
            btn = QPushButton(name)
            btn.setStyleSheet("QPushButton {font-size: 45pt; padding: 20px;}")
            if name == " Abort ":
                btn.setFixedHeight(500)
            btn.clicked.connect(lambda _, num=i: self.handleButtonClick(num))
            self.buttonLayout.addWidget(btn)
            self.buttons.append(btn)

        buttonWidget = QWidget()
        buttonWidget.setLayout(self.buttonLayout)
        self.layout.addWidget(buttonWidget, 0, 3, len(button_names) + 1, 1)

        # Timer
        self.timer = QTimer()
        self.timer.setInterval(300)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):
        try:
            if not x:
                return

            # Graph 1: Fuel pressures
            if all([PT_FUEL_UP, PT_FUEL_DOWN, PT_FUEL_THRESH_UP, PT_FUEL_THRESH_DOWN]):
                self.plotDataItems[0][0].setData(list(x), list(PT_FUEL_UP))
                self.plotDataItems[0][1].setData(list(x), list(PT_FUEL_DOWN))
                self.plotDataItems[0][2].setData(list(x), list(PT_FUEL_THRESH_UP))
                self.plotDataItems[0][3].setData(list(x), list(PT_FUEL_THRESH_DOWN))

            # Graph 2: LOX pressures
            if all([PT_LOX_UP, PT_LOX_DOWN, PT_LOX_THRESH_UP, PT_LOX_THRESH_DOWN]):
                self.plotDataItems[1][0].setData(list(x), list(PT_LOX_UP))
                self.plotDataItems[1][1].setData(list(x), list(PT_LOX_DOWN))
                self.plotDataItems[1][2].setData(list(x), list(PT_LOX_THRESH_UP))
                self.plotDataItems[1][3].setData(list(x), list(PT_LOX_THRESH_DOWN))

            # Graph 3: Injector pressure
            if PT_I:
                self.plotDataItems[2][0].setData(list(x), list(PT_I))

            # Graph 4: Upstream pressure
            if PT_FUEL_UP2:
                self.plotDataItems[3][0].setData(list(x), list(PT_FUEL_UP2))

            # Graph 5: Instabilities and Derivatives
            if all([INSTABILITY, DP_LOX, DP_FUEL, DP]):
                self.plotDataItems[4][0].setData(list(x), list(INSTABILITY))
                self.plotDataItems[4][1].setData(list(x), list(DP_LOX))
                self.plotDataItems[4][2].setData(list(x), list(DP_FUEL))
                self.plotDataItems[4][3].setData(list(x), list(DP))

            # Graph 6: States
            if FUEL_STATE and LOX_STATE:
                self.plotDataItems[5][0].setData(list(x), list(FUEL_STATE))
                self.plotDataItems[5][1].setData(list(x), list(LOX_STATE))

        except Exception as e:
            print(f"Plot update error: {e}")

    def handleButtonClick(self, number):
        try:
            esp32.write(("s" + str(number)).encode())
            print(f"Button {number} pressed")
        except Exception as e:
            print(f"Serial error: {e}")

def main():
    t1 = Thread(target=collection)
    t1.start()
    app = QApplication(sys.argv)
    main_window = LivePlotter()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
