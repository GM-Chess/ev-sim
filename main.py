import pandas as pd
import numpy as np
import functions as func
import variable as var
import matplotlib.pyplot as plt
import batteryClass as bc

class VehicleSimulation:
    def __init__(self):
        self.actual_speed = 0.0
        self.actual_acceleration = 0.0
        self.motor_torque = 0.0
        self.motor_current = 0.0
        self.motor_voltage = 0.0
        self.integral = 0.0
        
    def update_vehicle_dynamics(self, motor_torque, dt):
        """Update actual vehicle speed based on motor torque"""
        wheel_torque = motor_torque * var.gearRatio * 0.95
        traction_force = wheel_torque / var.wheelRadius
        
        # Resistance forces at CURRENT speed
        roll_force = func.fRoll(self.actual_speed)
        aero_force = func.fAerodynamic(self.actual_speed)
        total_resistance = roll_force + aero_force
        
        net_force = traction_force - total_resistance  # FIXED: resistance should oppose motion
        
        # Improved zero speed handling
        if abs(self.actual_speed) < 0.001 and net_force <= 0:
            # Vehicle is stopped and no positive force to move it
            self.actual_acceleration = 0
        else:
            # Normal acceleration calculation
            self.actual_acceleration = net_force / (var.vehicleMass * var.rotationalInertiaCoeff)
        
        # Update speed with numerical stability
        new_speed = self.actual_speed + self.actual_acceleration * dt
        self.actual_speed = max(0, new_speed)  # Prevent negative speed
        
        return self.actual_speed, self.actual_acceleration 

vehicle = VehicleSimulation()
batteryPack = bc.BatteryPack()

HWFETdata = pd.read_csv('HWFET.csv')
HWTime = HWFETdata['Time (seconds)'].values
HWSpeed = HWFETdata['Speed (mph)'].values / 2.237
HWaccDesired = np.gradient(HWSpeed, HWTime)
HWdistance = np.trapezoid(HWSpeed, HWTime)

HWresults = []

# FIX: Pre-calculate interpolation to avoid issues
interpolation_factor = 5  # Reduced for stability
total_interpolated_points = len(HWTime) * interpolation_factor
interpolated_time = np.linspace(HWTime[0], HWTime[-1], total_interpolated_points)
interpolated_desired_speed = np.interp(interpolated_time, HWTime, HWSpeed)

# Calculate dt for interpolated points
interp_dt = (HWTime[-1] - HWTime[0]) / (total_interpolated_points - 1)

print(f"Original time steps: {len(HWTime)}")
print(f"Interpolated time steps: {total_interpolated_points}")
print(f"Interpolated dt: {interp_dt:.4f}s")

for i in range(total_interpolated_points):
    desired_speed = interpolated_desired_speed[i]
    current_time = interpolated_time[i]
    
    # DEBUG: Track what's happening around problematic region
    if 710 <= current_time <= 720:
        print(f"\nStep {i}: Time={current_time:.1f}s, Desired={desired_speed:.2f}m/s, Actual={vehicle.actual_speed:.2f}m/s")
    
    V_ref, vehicle.integral = func.pi_controller(desired_speed, vehicle.actual_speed, vehicle.integral, interp_dt)
    speed_error = desired_speed - vehicle.actual_speed
    
    # Mode determination
    if speed_error > 0.1:
        mode = "DRIVING"
        V_batt = batteryPack.update_battery_voltage()
        V_motor, V_arm = func.buckConverter(batteryPack.soc, V_batt, V_ref)
    elif speed_error < -0.1:
        mode = "BRAKING" 
        V_batt = batteryPack.update_battery_voltage()
        V_motor, V_arm = func.regenerativeBrakeConverter(batteryPack.soc, V_batt, abs(V_ref))
    else:
        mode = "COASTING"
        V_batt = batteryPack.update_battery_voltage()
        V_motor, V_arm = func.buckConverter(batteryPack.soc, V_batt, V_ref)
    
    motor_angular_vel = func.angularVelocity(vehicle.actual_speed, var.gearRatio)
    
    # Calculate desired acceleration from interpolated speed profile
    if i == 0:
        acc_desired = 0
    else:
        acc_desired = (interpolated_desired_speed[i] - interpolated_desired_speed[i-1]) / interp_dt
    
    # FIX: Add safety check for angular velocity calculations
    wheel_angular_vel = func.angularVelocity(desired_speed, 1)
    if abs(wheel_angular_vel) < 0.001:  # Avoid division by near-zero
        required_wheel_torque = 0
    else:
        f_tr_desired = (var.rotationalInertiaCoeff * var.vehicleMass * acc_desired + 
                        func.fRoll(desired_speed) + 
                        func.fAerodynamic(desired_speed) + 
                        func.fgxt())
        
        required_power = func.powerFromForce(abs(f_tr_desired), desired_speed)
        required_wheel_torque = required_power / wheel_angular_vel
    
    required_motor_torque = required_wheel_torque / var.gearRatio
    
    # Apply mode-specific torque sign
    if mode == "BRAKING":
        required_motor_torque = -abs(required_motor_torque)
    else:
        required_motor_torque = abs(required_motor_torque)
    
    # Calculate motor current and torque with safety checks
    vehicle.motor_current = func.currentFromTorqueAndSpeed(
        motor_angular_vel, required_motor_torque, V_motor
    )
    
    # FIX: Add bounds checking for motor current/torque
    vehicle.motor_current = np.clip(vehicle.motor_current, -var.ratedArmatureCurrent, var.ratedArmatureCurrent)
    vehicle.motor_torque = vehicle.motor_current * var.motorConstant
    
    # Update vehicle dynamics
    actual_speed, actual_accel = vehicle.update_vehicle_dynamics(vehicle.motor_torque, interp_dt)
    
    # Update battery
    soc, pack_voltage, _ = batteryPack.update_battery(vehicle.motor_current, interp_dt)
    
    # DEBUG: Check for problematic states
    if 710 <= current_time <= 720:
        print(f"  Mode={mode}, MotorTorque={vehicle.motor_torque:.2f}Nm, "
              f"Accel={actual_accel:.3f}m/s², Speed={vehicle.actual_speed:.3f}m/s")
    
    # Store results at original time steps (for cleaner plotting)
    original_time_indices = np.searchsorted(HWTime, current_time, side='right') - 1
    if original_time_indices >= 0 and (i == total_interpolated_points - 1 or 
                                       abs(current_time - HWTime[original_time_indices]) < interp_dt/2):
        HWresults.append({
            'time': current_time,
            'desired_speed': desired_speed,  # FIX: This should now be correct
            'actual_speed': vehicle.actual_speed,
            'actual_acceleration': actual_accel,
            'motor_torque': vehicle.motor_torque,
            'motor_current': vehicle.motor_current,
            'motor_voltage': V_motor,
            'battery_voltage': pack_voltage,
            'soc': soc,
            'V_ref': V_ref,
            'mode': mode
        })

HWresults_df = pd.DataFrame(HWresults)

plt.figure()
plt.plot(HWresults_df['time'], HWresults_df['actual_speed'], label='Actual Speed', color='green')
plt.plot(HWresults_df['time'], HWresults_df['desired_speed'], label='Desired Speed', color='orange', linestyle='--')
plt.title('HW Vehicle Acceleration and Speed vs Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Acceleration (m/s²) / Speed (m/s)   ')
plt.legend()
plt.grid()

plt.figure()
plt.plot(HWresults_df['time'], HWresults_df['soc'], label='State of Charge (SOC)', color='blue')
plt.title('HW Battery State of Charge vs Time')
plt.xlabel('Time (seconds)')
plt.ylabel('SOC (%)')
plt.legend()
plt.grid()

plt.figure()
plt.plot(HWresults_df['time'], HWresults_df['battery_voltage'], label='Battery Voltage', color='red')
plt.title('HW Battery Voltage vs Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Battery Voltage (V)')
plt.legend()
plt.grid()

plt.figure()
plt.plot(HWresults_df['time'], HWresults_df['motor_current'], label='Battery Current', color='purple')
plt.title('HW Battery Current vs Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Battery Current (A)')
plt.legend()
plt.grid()
plt.show()

# UDDSvehicle = VehicleSimulation()
# UDDSbatteryPack = bc.BatteryPack()
# UDDSdata = pd.read_csv('UDDS.csv')
# UDDSTime = UDDSdata['Time (seconds)'].values
# UDDSSpeed = UDDSdata['Speed (mph)'].values / 2.237
# UDDSaccDesired = np.gradient(UDDSSpeed, UDDSTime)
# UDDSdistance = np.trapezoid(UDDSSpeed, UDDSTime)
# UDDSresults = []

# for i in range(len(UDDSTime)):
#     desired_speed = UDDSSpeed[i]
#     dt = 1
#     initial_actual_speed = UDDSvehicle.actual_speed
    
#     V_ref, UDDSvehicle.integral = func.pi_controller(desired_speed, UDDSvehicle.actual_speed, UDDSvehicle.integral, dt)
    
#     speed_error = desired_speed - UDDSvehicle.actual_speed
    
#     if speed_error > 0.1:
#         mode = "DRIVING"
#         V_batt = UDDSbatteryPack.update_battery_voltage()
#         V_motor, V_arm = func.buckConverter(UDDSbatteryPack.soc, V_batt, V_ref)
#         motor_current_sign = 1
        
#     elif speed_error < -0.1:
#         mode = "BRAKING"
#         V_batt = UDDSbatteryPack.update_battery_voltage()
#         V_motor, V_arm = func.regenerativeBrakeConverter(UDDSbatteryPack.soc, V_batt, abs(V_ref))
#         motor_current_sign = -1
        
#     else:
#         mode = "COASTING"
#         V_batt = UDDSbatteryPack.update_battery_voltage()
#         V_motor, V_arm = func.buckConverter(UDDSbatteryPack.soc, V_batt, V_ref)
#         motor_current_sign = 1
    
#     motor_angular_vel = func.angularVelocity(UDDSvehicle.actual_speed, var.gearRatio)
    
#     if i == 0:
#         acc_desired = 0
#     else:
#         acc_desired = (UDDSSpeed[i] - UDDSSpeed[i-1]) / dt

#     f_tr_desired = (var.rotationalInertiaCoeff * var.vehicleMass * acc_desired +
#                     func.fRoll(desired_speed) + 
#                     func.fAerodynamic(desired_speed) + 
#                     func.fgxt())
#     required_power = func.powerFromForce(abs(f_tr_desired), desired_speed)
#     wheel_angular_vel = func.angularVelocity(desired_speed, 1)
#     required_wheel_torque = required_power / wheel_angular_vel if wheel_angular_vel != 0 else 0
#     required_motor_torque = required_wheel_torque / var.gearRatio
#     if mode == "BRAKING":
#         required_motor_torque = -abs(required_motor_torque)
#     else:
#         required_motor_torque = abs(required_motor_torque)
#     vehicle.motor_current = func.currentFromTorqueAndSpeed(
#         motor_angular_vel, required_motor_torque, V_motor
#     )
#     vehicle.motor_torque = vehicle.motor_current * var.motorConstant
#     old_speed = UDDSvehicle.actual_speed
#     actual_speed, actual_accel = UDDSvehicle.update_vehicle_dynamics(UDDSvehicle.motor_torque, dt)
#     soc, pack_voltage, _ = UDDSbatteryPack.update_battery(UDDSvehicle.motor_current, dt)
#     UDDSresults.append({
#         "time": UDDSTime[i],
#         "desired_speed": desired_speed,
#         "actual_speed": actual_speed,
#         "motor_current": UDDSvehicle.motor_current,
#         "motor_torque": UDDSvehicle.motor_torque,
#         "soc": soc,
#         "pack_voltage": pack_voltage,
#         "mode": mode
#     })

# UDDSresults_df = pd.DataFrame(UDDSresults)
# plt.figure()    
# plt.plot(UDDSresults_df['time'], UDDSresults_df['actual_speed'], label='Actual Speed', color='green')
# plt.plot(UDDSresults_df['time'], UDDSresults_df['desired_speed'], label='Desired Speed', color='orange', linestyle='--')
# plt.title('UDDS Vehicle Speed vs Time')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Speed (m/s)')
# plt.legend()
# plt.grid(True)
  
# plt.figure()
# plt.plot(UDDSresults_df['time'], UDDSresults_df['soc'], label='State of Charge (SOC)', color='blue')
# plt.title('UDDS Battery State of Charge vs Time')
# plt.xlabel('Time (seconds)')
# plt.ylabel('SOC (%)')
# plt.legend()
# plt.grid(True)
  
# plt.figure()
# plt.plot(UDDSresults_df['time'], UDDSresults_df['pack_voltage'], label='Battery Voltage', color='red')
# plt.title('UDDS Battery Voltage vs Time')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Battery Voltage (V)')
# plt.legend()    
# plt.grid(True)
  
# plt.figure()
# plt.plot(UDDSresults_df['time'], UDDSresults_df['motor_current'], label='Battery Current', color='purple')
# plt.title('UDDS Battery Current vs Time')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Battery Current (A)')
# plt.legend()

# US06vehicle = VehicleSimulation()
# US06batteryPack = bc.BatteryPack()
# US06data = pd.read_csv('US06.csv')
# US06Time = US06data['Time (seconds)'].values
# US06Speed = US06data['Speed (mph)'].values / 2.237
# US06accDesired = np.gradient(US06Speed, US06Time)
# US06distance = np.trapezoid(US06Speed, US06Time)
# US06results = []

# for i in range(len(US06Time)):
#     desired_speed = US06Speed[i]
#     dt = 1
#     initial_actual_speed = US06vehicle.actual_speed
#     V_ref, US06vehicle.integral = func.pi_controller(desired_speed, US06vehicle.actual_speed, US06vehicle.integral, dt)
#     speed_error = desired_speed - US06vehicle.actual_speed

#     if speed_error > 0.1:
#         mode = "DRIVING"
#         V_batt = US06batteryPack.update_battery_voltage()
#         V_motor, V_arm = func.buckConverter(US06batteryPack.soc, V_batt, V_ref)
#         motor_current_sign = 1

#     elif speed_error < -0.1:
#         mode = "BRAKING"
#         V_batt = US06batteryPack.update_battery_voltage()
#         V_motor, V_arm = func.regenerativeBrakeConverter(US06batteryPack.soc, V_batt, abs(V_ref))
#         motor_current_sign = -1

#     else:
#         mode = "COASTING"
#         V_batt = US06batteryPack.update_battery_voltage()
#         V_motor, V_arm = func.buckConverter(US06batteryPack.soc, V_batt, V_ref)
#         motor_current_sign = 1

#     motor_angular_vel = func.angularVelocity(US06vehicle.actual_speed, var.gearRatio)
#     if i == 0:
#         acc_desired = 0
#     else:
#         acc_desired = (US06Speed[i] - US06Speed[i-1]) / dt

#     f_tr_desired = -(var.rotationalInertiaCoeff * var.vehicleMass * acc_desired +
#                     func.fRoll(desired_speed) +
#                     func.fAerodynamic(desired_speed) +
#                     func.fgxt())
#     print(f_tr_desired)
#     print(func.fRoll(desired_speed), 
#           func.fAerodynamic(desired_speed), 
#           func.fgxt(),
#           var.rotationalInertiaCoeff * var.vehicleMass * acc_desired)
#     required_power = func.powerFromForce(abs(f_tr_desired), desired_speed)
#     wheel_angular_vel = func.angularVelocity(desired_speed, 1)
#     required_wheel_torque = required_power / wheel_angular_vel if wheel_angular_vel != 0 else 0
#     required_motor_torque = required_wheel_torque / var.gearRatio
#     if mode == "BRAKING":
#         required_motor_torque = -abs(required_motor_torque)
#     else:
#         required_motor_torque = abs(required_motor_torque)

#     US06vehicle.motor_current = func.currentFromTorqueAndSpeed(
#         motor_angular_vel, required_motor_torque, V_motor
#     )
#     US06vehicle.motor_torque = US06vehicle.motor_current * var.motorConstant
#     old_speed = US06vehicle.actual_speed
#     actual_speed, actual_accel = US06vehicle.update_vehicle_dynamics(US06vehicle.motor_torque, dt)
#     soc, pack_voltage, _ = US06batteryPack.update_battery(US06vehicle.motor_current, dt)
#     US06results.append({
#         "time": US06Time[i],
#         "desired_speed": desired_speed,
#         "actual_speed": actual_speed,
#         "desired_acceleration": acc_desired,
#         "actual_acceleration": actual_accel,
#         "motor_current": vehicle.motor_current,
#         "motor_torque": vehicle.motor_torque,
#         "battery_soc": soc,
#         "battery_voltage": pack_voltage,
#         "mode": mode
#     })

# US06results_df = pd.DataFrame(US06results)

# plt.figure()
# plt.plot(US06results_df['time'], US06results_df['actual_speed'], label='Actual Speed', color='green')
# plt.plot(US06results_df['time'], US06results_df['desired_speed'], label='Desired Speed', color='orange', linestyle='--')
# plt.title('US06 Vehicle Speed vs Time')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Speed (m/s)')
# plt.legend()
# plt.grid(True)

# plt.figure()
# plt.plot(US06results_df['time'], US06results_df['battery_soc'], label='State of Charge (SOC)', color='blue')
# plt.title('US06 Battery State of Charge vs Time')
# plt.xlabel('Time (seconds)')
# plt.ylabel('SOC (%)')
# plt.legend()
# plt.grid(True)

# plt.figure()
# plt.plot(US06results_df['time'], US06results_df['battery_voltage'], label='Battery Voltage', color='red')
# plt.title('US06 Battery Voltage vs Time')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Battery Voltage (V)')
# plt.legend()
# plt.grid(True)

# plt.figure()
# plt.plot(US06results_df['time'], US06results_df['motor_current'], label='Battery Current', color='purple')
# plt.title('US06 Battery Current vs Time')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Battery Current (A)')
# plt.legend()
# plt.grid(True)

# x = input("press enter to print all graphs ... ")
# if x == "":
#     plt.close('all')

# print("Simulation completed and plots generated.")
# print(f"Total distance traveled in HWFET cycle: {HWdistance} meters")
# print(f"Total distance traveled in UDDS cycle: {UDDSdistance} meters")
# print(f"Total distance traveled in US06 cycle: {US06distance} meters")
