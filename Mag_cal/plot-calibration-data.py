"""

PLOT UNCALIBRTED AND CALIBRATED MAGNETOMETER MEASUREMENTS

Read data file with raw measurements and compare calibrated vs. uncalibrated
measurements.


The calibration equation is:
    h_calib = A * (h_meas - b)

h_calib: Calibrated measurements (3x1)
A: Soft iron, scale factor, and misalignment correction matrix (3x3, symmetric)
h_meas: Raw measurements (3x1)
b: Hard-iron offset correction vector (3x1)

Calibration parametrers were determined by the Magneto calibration software (see below).

Reference
---------
    Based on:
        Michael Wrona's Blog
        https://www.youtube.com/watch?v=MinV5V1ioWg&t=297s

    Magneto magnetometer calibration software download:
        https://sites.google.com/site/sailboatinstruments1/home

"""


import numpy as np
import matplotlib.pyplot as plt


# Define calibration parameters
A = np.array(   [[1.102381,0.021149,0.032939],
                [0.021149,1.007658, -0.108953],
                [0.032939, -0.108953,1.119184]])
b = np.array([69.127276,-59.899396, 161.283932])


# Read raw data and apply calibration
rawData = np.genfromtxt('mag-readings.txt', delimiter='\t')  # Read raw measurements

N = len(rawData)
calibData = np.zeros((N, 3), dtype='float')
for i in range(N):
    currMeas = np.array([rawData[i, 0], rawData[i, 1], rawData[i, 2]])
    calibData[i, :] = A @ (currMeas - b)


# Plot XY data
plt.figure()
plt.plot(rawData[:, 0], rawData[:, 1], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 0], calibData[:, 1], 'r*', label='Calibrated Meas.')
plt.title('XY Magnetometer Data')
plt.xlabel('X [uT]')
plt.ylabel('Y [uT]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot YZ data
plt.figure()
plt.plot(rawData[:, 1], rawData[:, 2], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 1], calibData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('YZ Magnetometer Data')
plt.xlabel('Y [uT]')
plt.ylabel('Z [uT]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot XZ data
plt.figure()
plt.plot(rawData[:, 0], rawData[:, 2], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 0], calibData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('XZ Magnetometer Data')
plt.xlabel('X [uT]')
plt.ylabel('Z [uT]')
plt.legend()
plt.grid()
plt.axis('equal')


# Plot 3D scatter
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(N):
    xraw = rawData[i, 0]
    yraw = rawData[i, 1]
    zraw = rawData[i, 2]

    xcalib = calibData[i, 0]
    ycalib = calibData[i, 1]
    zcalib = calibData[i, 2]
    ax.scatter(xraw, yraw, zraw, color='r')
    ax.scatter(xcalib, ycalib, zcalib, color='b')

ax.set_title('3D Scatter Plot of Magnetometer Data')
ax.set_xlabel('X [uT]')
ax.set_ylabel('Y [uT]')
ax.set_zlabel('Z [uT]')


plt.show()