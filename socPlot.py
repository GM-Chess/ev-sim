import matplotlib.pyplot as plt
import numpy as np  

soc = [100, 90, 80, 70, 60, 50, 40, 30, 20, 10]
operatingVoltage = [6.37, 6.31, 6.25, 6.19, 6.12, 6.05, 5.98, 5.91, 5.83, 5.75]

plt.plot(soc, operatingVoltage, marker='o')
plt.title('State of Charge (SOC) vs Operating Voltage (OCV)')
plt.xlabel('State of Charge (%)')
plt.ylabel('Operating Voltage (V)')
plt.grid()
plt.show()