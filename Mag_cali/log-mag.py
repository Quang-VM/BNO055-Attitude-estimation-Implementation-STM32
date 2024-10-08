import csv
import time
import serial
import numpy as np
import matplotlib.pyplot as plt

# GLOBAL VARIABLES
SER_PORT = 'COM19'  # Serial port
SER_BAUD = 115200  # Serial baud rate
SAMPLE_FREQ = 50  # Frequency to record magnetometer readings at [Hz]
T_SAMPLE = 200  # Total time to read magnetometer readings [sec]
OUTPUT_FILENAME = 'mag-readings.txt'  # Output data file name

class SerialPort:
    """Create and read data from a serial port."""

    def __init__(self, port, baud=115200):
        """Create and read serial data."""
        if not isinstance(port, str):
            raise TypeError('port must be a string.')

        if not isinstance(baud, int):
            raise TypeError('Baud rate must be an integer.')

        self.port = port
        self.baud = baud

        # Initialize serial connection
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        self.ser.flushInput()
        self.ser.flushOutput()

    def read(self, clean_end=True):
        """Read and decode data string from serial port."""
        bytesToRead = self.ser.readline()
        decodedMsg = bytesToRead.decode('utf-8')

        if clean_end:
            decodedMsg = decodedMsg.strip('\r\n')  # Strip '\r\n' characters
        
        return decodedMsg

    def write(self, msg):
        """Write string to serial port."""
        try:
            self.ser.write(msg.encode())
            return True
        except Exception as e:
            print(f"Error sending message: {e}")
            return False

    def close(self):
        """Close serial connection."""
        self.ser.close()

class PlotPoints3D:
    """Plot magnetometer readings as 3D scatter plot."""

    def __init__(self, fig, ax, live_plot=True, marker='o', c='r'):
        """Initialize 3D scatter plot."""
        self.fig = fig
        self.ax = ax
        self.live_plot = live_plot
        self.ptMarker = marker
        self.ptColor = c
        self.edgeColor = 'k'
        self.ax.set_xlim((-80, 80))
        self.ax.set_ylim((-80, 80))
        self.ax.set_zlim((-80, 80))

    def add_point(self, x, y, z):
        """Add a 3D point to the scatter plot."""
        self.ax.scatter(x, y, z, marker=self.ptMarker,
                        c=self.ptColor, edgecolors=self.edgeColor)
        if self.live_plot:
            plt.pause(0.001)

def main():
    Serial = SerialPort(SER_PORT, SER_BAUD)
    n = int(SAMPLE_FREQ * T_SAMPLE)  # Number of readings
    dt = 1.0 / SAMPLE_FREQ  # Sample period [sec]

    # Create live plot for logging points
    fig_raw_readings = plt.figure()
    ax_raw_readings = fig_raw_readings.add_subplot(111, projection='3d')
    raw_data_plot = PlotPoints3D(fig_raw_readings, ax_raw_readings, live_plot=False)

    # Take a few readings to 'flush' out bad ones
    for _ in range(4):
        data = Serial.read().split(',')  # Split into separate values
        time.sleep(0.25)

    measurements = np.zeros((n, 3), dtype='float')

    for curr_meas in range(n):
        data = Serial.read().split(',')  # Split into separate values

        try:
            mx, my, mz = float(data[0]), float(data[1]), float(data[2])  # Convert to floats, [uT]
        except ValueError:
            print(f"Error reading data at measurement {curr_meas + 1}. Skipping this measurement.")
            continue

        print(f'[{mx:.4f}, {my:.4f}, {mz:.4f}] uT  |  Norm: {np.sqrt(mx**2 + my**2 + mz**2):.4f} uT  |  {curr_meas / n * 100.0:.1f} % Complete.')

        # Store data in array
        measurements[curr_meas, 0] = mx
        measurements[curr_meas, 1] = my
        measurements[curr_meas, 2] = mz

        raw_data_plot.add_point(mx, my, mz)  # Add point to 3D plot

    # After measurements, write data to file
    Serial.close()
    print('Sensor Reading Complete!')

    print(f'Writing data to {OUTPUT_FILENAME} ...')
    with open(OUTPUT_FILENAME, 'w', newline='') as f:
        writer = csv.writer(f, delimiter='\t')
        writer.writerows(measurements)

    plt.show()

if __name__ == "__main__":
    main()
