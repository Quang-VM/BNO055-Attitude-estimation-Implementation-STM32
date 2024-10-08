"""
Display CSV-formatted serial data 
Should work with format %f,%f,%f,%f\n

Reference:

    Jon Froehlich
        http://makeabilitylab.io

    Based on:
        - https://electronut.in/plotting-real-time-data-from-arduino-using-python/ by Mahesh Venkitachalam
        - https://www.thepoorengineer.com/en/arduino-python-plot/ 

"""

import sys, serial, argparse
import numpy as np
from time import sleep
from collections import deque
import math

import matplotlib.pyplot as plt
import matplotlib.animation as animation


# plot class
class SerialPlot:
    # constr
    def __init__(self, num_values_to_plot, str_port, baud_rate=115200, max_length=100):
        # open serial port
        self.ser = serial.Serial(str_port, 115200)

        self.data = list()
        for i in range(0, num_values_to_plot):
            buf = deque([0.0] * max_length)
            self.data.append(buf)

        self.max_length = max_length

    # update plot
    def update(self, frameNum, args, plt_lines):
        try:
            while self.ser.in_waiting:
                new_line = self.ser.readline()
                new_line = new_line.decode('utf-8')
                new_data = new_line.split(",")
                new_data = [float(val.strip()) for val in new_line.split(",")]
                print(new_data)

                start_idx = 0
                if args.includes_timestamp:
                    # todo: in future could also buffer timestamp and plot it on x-axis
                    start_idx = 1

                buf_idx = 0
                for i in range(start_idx, len(new_data)):
                    val = new_data[i]
                    buf = self.data[buf_idx]
                    if len(buf) < self.max_length:
                        buf.append(val)
                    else:
                        # start scrolling
                        buf.popleft()
                        buf.append(val)

                    buf_idx += 1


                for i in range(0, len(plt_lines)):
                    plt_lines[i].set_data(range(self.max_length), self.data[i])

        except KeyboardInterrupt:
            print('exiting')

        except Exception as e:
            print('Error '+ str(e))

        #return a0,
        return None

    # clean up
    def close(self):
        # close serial
        self.ser.flush()
        self.ser.close()

    # main() function


def main():
	# python serial_plotter.py --port /dev/cu.usbmodem14601
	# windows: python Serial_plot.py --port COM19	
    # create parser
    parser = argparse.ArgumentParser(description="Serial Plotter")

    # add expected arguments
    parser.add_argument('--port', dest='port', required=True, help='the serial port for incoming data')
    parser.add_argument('--num_vals', dest='num_vals', required=False, default=4, type=int, 
        help='num vals in CSV (not counting the timestamp column, if it exists')
    parser.add_argument('--max_len', dest='max_len', required=False, default=500, type=int, 
        help='the number of samples to plot at a time')
    parser.add_argument('--includes_timestamp', dest='includes_timestamp', type=bool, required=False, default=False, 
        help="is the incoming csv's first column a timestamp?")

    # parse args
    args = parser.parse_args()

    # strPort = '/dev/tty.usbserial-A7006Yqh'
    str_port = str(args.port)

    print('Reading from serial port: {}'.format(str_port))

    # plot parameters
    serial_plot = SerialPlot(args.num_vals, str_port, max_length=args.max_len)

    # set up animation
    fig = plt.figure(figsize=(10, 10))
    ax = plt.axes(xlim=(0, args.max_len), ylim=(-90, 120))

    lines = list()
    # label
    labels = ['Yaw (Z)', 'Pitch (Y)', 'Roll (X)', 'Mag Norm']
    for i in range(0, args.num_vals):
        if i == 3:  # Đường thứ 4 (Mag Norm)
            line2d, = ax.plot([], [], label=labels[i], linestyle='--', color='black')  
        else:
            line2d, = ax.plot([], [], label=labels[i]) 
        lines.append(line2d)

   
    ax.set_title("MADGWICK ALGORITHM", fontsize=22)
    ax.set_xlabel('')
    ax.set_ylabel('Degrees')

    plt.legend(loc='upper right')  
    anim = animation.FuncAnimation(fig, serial_plot.update,
                                fargs=(args, lines),
                                interval=30)

    # show plot
    plt.show()

    # clean up
    serial_plot.close()




    # clean up
    serial_plot.close()

    print('Exiting...')


# call main
if __name__ == '__main__':
    main()