#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np 

file_basepath = "/home/vpreston/biolab_ws/src/london_simulation/tut_arrows/output/x_simulation/_2016-11-04-14-50-56"
data_depth = np.genfromtxt(file_basepath+"/_slash_ucat0_slash_tracking.csv", delimiter=",", names=True)
data_surge = np.genfromtxt(file_basepath+"/_slash_ucat0_slash_tracking_x.csv", delimiter=",", names=True) 
data_sway = np.genfromtxt(file_basepath+"/_slash_ucat0_slash_tracking_y.csv", delimiter=",", names=True)
data_yaw = np.genfromtxt(file_basepath+"/_slash_ucat0_slash_tracking_yaw.csv", delimiter=",", names=True)
data_pitch = np.genfromtxt(file_basepath+"/_slash_ucat0_slash_tracking_pitch.csv", delimiter=",", names=True)
data_roll = np.genfromtxt(file_basepath+"/_slash_ucat0_slash_tracking_roll.csv", delimiter=",", names=True)


def get_data(data_struct):
	x = data_struct['secs'] + data_depth['nsecs'] / 1e9
	actual = data_struct['x']
	commanded = data_struct['y']
	control = data_struct['z']
	return [x, actual, commanded, control]

def plot_data(data_struct, title, label, key, colors, axes):
	x, actual, commanded, control = get_data(data_struct)
	axes[0].set_title(title)
	axes[0].set_xlabel(label[0])
	axes[0].set_ylabel(label[1])
	axes[0].plot(x,actual, c=colors[0], label=key[0])
	axes[0].plot(x,commanded, c=colors[1], label=key[1])
	
	axes[1].set_xlabel(label[0])
	axes[1].set_ylabel(label[2])
	axes[1].plot(x,control, c=colors[2], label=key[2])

	h1, l1 = axes[0].get_legend_handles_labels()
	axes[0].legend(h1, l1)
	h2, l2 = axes[1].get_legend_handles_labels()
	axes[1].legend(h2, l2)
	
#Depth
fig = plt.figure()
fig.suptitle("Cruise Mode, Surge Open Loop, 1.0")
ax1 = fig.add_subplot(261)
ax2 = fig.add_subplot(267)
plot_data(data_depth, "Depth", ["Time(s)", "Depth(m)", "Signal(N)"], ["Performed","Commanded","Control"], ["r", "b", "g"], [ax1, ax2])

#Surge
ax3 = fig.add_subplot(262)
ax4 = fig.add_subplot(268)
plot_data(data_surge, "Surge", ["Time(s)", "Surge(m)", "Signal(N)"], ["Performed","Commanded","Control"], ["r", "b", "g"], [ax3, ax4])

#Sway
ax5 = fig.add_subplot(263)
ax6 = fig.add_subplot(269)
plot_data(data_sway, "Sway", ["Time(s)", "Surge(m)", "Signal(N)"], ["Performed","Commanded","Control"], ["r", "b", "g"], [ax5, ax6])

#Roll
ax7 = fig.add_subplot(264)
ax8 = fig.add_subplot(2,6,10)
plot_data(data_roll, "Roll", ["Time(s)", "Surge(m)", "Signal(N)"], ["Performed","Commanded","Control"], ["r", "b", "g"], [ax7, ax8])

#Pitch
ax9 = fig.add_subplot(265)
ax10 = fig.add_subplot(2,6,11)
plot_data(data_pitch, "Pitch", ["Time(s)", "Surge(m)", "Signal(N)"], ["Performed","Commanded","Control"], ["r", "b", "g"], [ax9, ax10])

#Yaw
ax11 = fig.add_subplot(266)
ax12 = fig.add_subplot(2,6,12)
plot_data(data_yaw, "Yaw", ["Time(s)", "Surge(m)", "Signal(N)"], ["Performed","Commanded","Control"], ["r", "b", "g"], [ax11, ax12])

plt.show()
