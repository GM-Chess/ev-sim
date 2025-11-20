import matplotlib.pyplot as plt
import numpy as np  
import pandas as pd

#open and read the CSV files for time and speed data
HWFETdata = pd.read_csv('HWFET.csv')
HWTime = HWFETdata['Time (seconds)'].values  # assuming 'Time' column
HWSpeed = HWFETdata['Speed (mph)'].values  # assuming 'Speed' column

UDDSdata = pd.read_csv('UDDS.csv')
UDDSTime = UDDSdata['Time (seconds)'].values  # assuming 'Time
UDDSSpeed = UDDSdata['Speed (mph)'].values  # assuming 'Speed' column

US06data = pd.read_csv('US06.csv')
US06Time = US06data['Time (seconds)'].values  # assuming 'Time
US06Speed = US06data['Speed (mph)'].values  # assuming 'Speed' column

plt.figure(figsize=(12, 8))
plt.plot(HWTime, HWSpeed, label='HWFET', color='blue')
plt.plot(UDDSTime, UDDSSpeed, label='UDDS', color='orange')
plt.plot(US06Time, US06Speed, label='US06', color='green')
plt.title('Speed vs Time for Different Driving Cycles')
plt.xlabel('Time (seconds)')
plt.ylabel('Speed (mph)')
plt.legend()
plt.grid()
plt.show()
print("Speed vs Time plots generated.")