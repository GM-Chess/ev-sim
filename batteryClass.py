import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

class BatteryPack:
    def __init__(self):
        
        self.cell_voltage_nominal = 6.0  
        self.cell_capacity = 225  
        self.cells_series = 50
    
        
        
        self.pack_voltage_nominal = self.cell_voltage_nominal * self.cells_series  
        self.pack_capacity = self.cell_capacity  
        
        self.soc_table = np.array([10, 20, 30, 40, 50, 60, 70, 80, 90, 100])
        self.ocv_table = np.array([5.75, 5.83, 5.91, 5.98, 6.05, 6.12, 6.19, 6.25, 6.31, 6.37])
        
        
        self.ocv_from_soc = interp1d(self.soc_table, self.ocv_table, kind='linear', fill_value='extrapolate')
        self.R_internal = 0.005 
        
        self.initial_soc = 100 
        self.current_ah = self.initial_soc / 100 * self.cell_capacity  
        self.soc = self.initial_soc  
    
    def soc_to_ocv(self, soc):
        return float(self.ocv_from_soc(soc))
    
    def update_battery(self, current, dt):
        cell_current = current
        ah_used = (cell_current * dt) / 3600 
        self.current_ah -= ah_used
    
        self.soc = (self.current_ah / self.cell_capacity) * 100  
        
        self.soc = np.clip(self.soc, 0, 100)
        self.current_ah = (self.soc / 100) * self.cell_capacity
        
        cell_ocv = self.soc_to_ocv(self.soc)
        cell_voltage = cell_ocv - (cell_current * self.R_internal)
        pack_voltage = cell_voltage * self.cells_series
        
        return self.soc, pack_voltage, cell_current
    
    def update_battery_voltage(self):
        cell_ocv = self.soc_to_ocv(self.soc)
        pack_voltage = cell_ocv * self.cells_series
        return pack_voltage
    
    def get_pack_voltage(self, current, soc=None):
     
        if soc is None:
            soc = self.soc
        
        cell_ocv = self.soc_to_ocv(soc)
        cell_voltage = cell_ocv - (current * self.R_internal)
        pack_voltage = cell_voltage * self.cells_series
        
        return pack_voltage
    
    def reset(self, initial_soc=100):
        self.initial_soc = initial_soc
        self.current_ah = initial_soc / 100 * self.cell_capacity
        self.soc = initial_soc 