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
        roll_force = -func.fRoll(self.actual_speed)  # NEGATIVE because they oppose motion
        aero_force = -func.fAerodynamic(self.actual_speed)  # NEGATIVE
        total_resistance = roll_force + aero_force
        
        net_force = traction_force + total_resistance
        
        # Check for numerical issues
        if np.isnan(net_force) or np.isinf(net_force):
            print(f"ERROR: net_force is {net_force}")
            print(f"  traction_force={traction_force}, roll={roll_force}, aero={aero_force}")
            print(f"  motor_torque={motor_torque}, speed={self.actual_speed}")
            return self.actual_speed, 0.0
        
        if self.actual_speed == 0 and net_force <= 0:
            self.actual_acceleration = 0
        else:
            self.actual_acceleration = net_force / (var.vehicleMass * var.rotationalInertiaCoeff)
        
        # Update speed
        new_speed = self.actual_speed + self.actual_acceleration * dt
        self.actual_speed = max(0, new_speed)
        
        return self.actual_speed, self.actual_acceleration    

vehicle = VehicleSimulation()
batteryPack = bc.BatteryPack()

HWFETdata = pd.read_csv('HWFET.csv')
HWTime = HWFETdata['Time (seconds)'].values
HWSpeed = HWFETdata['Speed (mph)'].values / 2.237
HWdistance = np.trapezoid(HWSpeed, HWTime)

HWresults = []
DEBUG = False  # Set to False to disable debug prints

for i in range(max(len(HWTime), 50)):  # Limit to 50 for testing
    desired_speed = HWSpeed[i]
    
    if i == 0:
        dt = HWTime[0]
    else:
        dt = HWTime[i] - HWTime[i-1]
    
    # Create interpolated substeps
    lineSpaceRange = np.linspace(HWTime[i - 1] if i > 0 else 0, HWTime[i], 100)
    HWyInterp = np.interp(lineSpaceRange, HWTime, HWSpeed)
    dt_substep = dt / len(lineSpaceRange)
    
    if DEBUG and i % 5 == 0:
        print(f"\n{'='*70}")
        print(f"TIME STEP {i}: t={HWTime[i]:.1f}s, desired_speed={desired_speed:.2f} m/s")
        print(f"  Current actual_speed={vehicle.actual_speed:.2f} m/s")
        print(f"  dt={dt:.3f}s, dt_substep={dt_substep:.6f}s, num_substeps={len(lineSpaceRange)}")
    
    # Track cumulative torque application
    total_torque_impulse = 0
    substep_count = {'DRIVING': 0, 'BRAKING': 0, 'COASTING': 0}
    
    for j in range(len(HWyInterp)):
        # PI controller generates voltage reference
        V_ref, vehicle.integral = func.pi_controller(
            HWyInterp[j], vehicle.actual_speed, vehicle.integral, dt_substep
        )
        
        speed_error = HWyInterp[j] - vehicle.actual_speed
        
        # Get battery voltage
        V_batt = batteryPack.update_battery_voltage()
        
        # Determine mode and get motor voltage
        if speed_error > 0.1:
            mode = "DRIVING"
            V_motor, V_arm = func.buckConverter(batteryPack.soc, V_batt, V_ref)
        elif speed_error < -0.1:
            mode = "BRAKING"
            V_motor, V_arm = func.regenerativeBrakeConverter(
                batteryPack.soc, V_batt, abs(V_ref)
            )
        else:
            mode = "COASTING"
            V_motor = 0
            V_arm = 0
        
        substep_count[mode] += 1
        
        # CRITICAL FIX: If V_motor is 0, we cannot produce torque
        if abs(V_motor) < 0.1:
            motor_current = 0
            motor_torque = 0
        else:
            motor_angular_vel = func.angularVelocity(vehicle.actual_speed, var.gearRatio)
            
            # Calculate desired acceleration for THIS substep
            if j == 0:
                acc_desired = 0
            else:
                acc_desired = (HWyInterp[j] - HWyInterp[j-1]) / dt_substep
            
            # CRITICAL FIX: Resistance forces are NEGATIVE (oppose motion)
            f_tr_desired = (var.rotationalInertiaCoeff * var.vehicleMass * acc_desired + 
                            func.fRoll(HWyInterp[j]) + 
                            func.fAerodynamic(HWyInterp[j]))
            
            required_power = func.powerFromForce(abs(f_tr_desired), HWyInterp[j])
            wheel_angular_vel = func.angularVelocity(HWyInterp[j], 1)
            required_wheel_torque = required_power / wheel_angular_vel if wheel_angular_vel > 0.01 else 0
            required_motor_torque = required_wheel_torque / var.gearRatio
            
            # Apply sign based on mode
            if mode == "BRAKING":
                required_motor_torque = -abs(required_motor_torque)
            else:
                required_motor_torque = abs(required_motor_torque)
            
            # Calculate actual achievable current and torque
            motor_current = func.currentFromTorqueAndSpeed(
                motor_angular_vel, required_motor_torque, V_motor
            )
            motor_torque = motor_current * var.motorConstant
            
            # Check for NaN/Inf
            if np.isnan(motor_torque) or np.isinf(motor_torque):
                print(f"ERROR at i={i}, j={j}: motor_torque is {motor_torque}")
                print(f"  motor_current={motor_current}, V_motor={V_motor}")
                print(f"  required_motor_torque={required_motor_torque}")
                motor_torque = 0
                motor_current = 0
        
        # CRITICAL FIX: Update vehicle dynamics at EACH substep!
        actual_speed, actual_accel = vehicle.update_vehicle_dynamics(
            motor_torque, dt_substep
        )
        
        # Update battery for this substep
        soc, pack_voltage, _ = batteryPack.update_battery(motor_current, dt_substep)
        
        total_torque_impulse += motor_torque * dt_substep
        
        # Store the final values from this substep
        vehicle.motor_current = motor_current
        vehicle.motor_torque = motor_torque
        vehicle.motor_voltage = V_motor
        
        # Check for runaway
        if vehicle.actual_speed > 100:  # 100 m/s = 360 km/h, clearly wrong
            print(f"\nERROR: Speed runaway detected at i={i}, j={j}")
            print(f"  Speed={vehicle.actual_speed:.2f} m/s")
            print(f"  motor_torque={motor_torque:.2f} Nm")
            print(f"  mode={mode}, V_motor={V_motor:.2f} V")
            raise ValueError("Simulation diverged - speed runaway")
    
    # Debug output for this major time step
    if DEBUG and i % 5 == 0:
        print(f"\n  Substep summary: {substep_count}")
        print(f"  Total torque impulse: {total_torque_impulse:.2f} Nm·s")
        print(f"  Final speed: {vehicle.actual_speed:.2f} m/s")
        print(f"  Final acceleration: {vehicle.actual_acceleration:.3f} m/s²")
        print(f"  Speed error: {desired_speed - vehicle.actual_speed:.2f} m/s")
        print(f"  Battery SOC: {soc:.1f}%")
    
    # Save results for this major time step
    HWresults.append({
        'time': HWTime[i],
        'desired_speed': desired_speed,
        'actual_speed': vehicle.actual_speed,
        'actual_acceleration': vehicle.actual_acceleration,
        'motor_torque': vehicle.motor_torque,
        'motor_current': vehicle.motor_current,
        'motor_voltage': vehicle.motor_voltage,
        'battery_voltage': pack_voltage,
        'soc': soc,
        'V_ref': V_ref,
        'mode': mode
    })

print("\n" + "="*70)
print("Simulation completed successfully!")
print(f"Final speed: {vehicle.actual_speed:.2f} m/s")
print(f"Final SOC: {soc:.1f}%")

HWresults_df = pd.DataFrame(HWresults)

# Plot results
plt.figure(figsize=(12, 8))

plt.subplot(2, 2, 1)
plt.plot(HWresults_df['time'], HWresults_df['actual_speed'], label='Actual Speed', color='green', linewidth=2)
plt.plot(HWresults_df['time'], HWresults_df['desired_speed'], label='Desired Speed', color='orange', linestyle='--', linewidth=2)
plt.title('Vehicle Speed vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 2)
plt.plot(HWresults_df['time'], HWresults_df['soc'], label='SOC', color='blue', linewidth=2)
plt.title('Battery State of Charge')
plt.xlabel('Time (s)')
plt.ylabel('SOC (%)')
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 3)
plt.plot(HWresults_df['time'], HWresults_df['motor_torque'], label='Motor Torque', color='red', linewidth=2)
plt.title('Motor Torque')
plt.xlabel('Time (s)')
plt.ylabel('Torque (Nm)')
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 4)
plt.plot(HWresults_df['time'], HWresults_df['motor_current'], label='Motor Current', color='purple', linewidth=2)
plt.title('Motor Current')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

print(f"\nDistance traveled: {np.trapezoid(HWresults_df['actual_speed'], HWresults_df['time']):.2f} meters")

UDDSVehicleSimulation = VehicleSimulation()
UDDSBatteryPack = bc.BatteryPack()

UDDSdata = pd.read_csv('UDDS.csv')
UDDSTime = UDDSdata['Time (seconds)'].values
UDDSSpeed = UDDSdata['Speed (mph)'].values / 2.237
UDDSdistance = np.trapezoid(UDDSSpeed, UDDSTime)
UDDSresults = []
DEBUG = False  # Set to False to disable debug prints
for i in range(max(len(UDDSTime), 50)):  # Limit to 50 for testing
    desired_speed = UDDSSpeed[i]
    
    if i == 0:
        dt = UDDSTime[0]
    else:
        dt = UDDSTime[i] - UDDSTime[i-1]
    
    # Create interpolated substeps
    lineSpaceRange = np.linspace(UDDSTime[i - 1] if i > 0 else 0, UDDSTime[i], 1000)
    UDDSyInterp = np.interp(lineSpaceRange, UDDSTime, UDDSSpeed)
    dt_substep = dt / len(lineSpaceRange)
    
    if DEBUG and i % 5 == 0:
        print(f"\n{'='*70}")
        print(f"TIME STEP {i}: t={UDDSTime[i]:.1f}s, desired_speed={desired_speed:.2f} m/s")
        print(f"  Current actual_speed={UDDSVehicleSimulation.actual_speed:.2f} m/s")
        print(f"  dt={dt:.3f}s, dt_substep={dt_substep:.6f}s, num_substeps={len(lineSpaceRange)}")
    
    # Track cumulative torque application
    total_torque_impulse = 0
    substep_count = {'DRIVING': 0, 'BRAKING': 0, 'COASTING': 0}
    
    for j in range(len(UDDSyInterp)):
        # PI controller generates voltage reference
        V_ref, UDDSVehicleSimulation.integral = func.pi_controller(
            UDDSyInterp[j], UDDSVehicleSimulation.actual_speed, UDDSVehicleSimulation.integral, dt_substep
        )
        
        speed_error = UDDSyInterp[j] - UDDSVehicleSimulation.actual_speed
        
        # Get battery voltage
        V_batt = UDDSBatteryPack.update_battery_voltage()
        
        # Determine mode and get motor voltage
        if speed_error > 0.1:
            mode = "DRIVING"
            V_motor, V_arm = func.buckConverter(UDDSBatteryPack.soc, V_batt, V_ref)
        elif speed_error < -0.1:
            mode = "BRAKING"
            V_motor, V_arm = func.regenerativeBrakeConverter(
                UDDSBatteryPack.soc, V_batt, abs(V_ref)
            )
        else:
            mode = "COASTING"
            V_motor = 0
            V_arm = 0
        substep_count[mode] += 1
        # CRITICAL FIX: If V_motor is 0, we cannot produce torque
        if abs(V_motor) < 0.1:
            motor_current = 0
            motor_torque = 0
        else:
            motor_angular_vel = func.angularVelocity(UDDSVehicleSimulation.actual_speed, var.gearRatio)
            
            # Calculate desired acceleration for THIS substep
            if j == 0:
                acc_desired = 0
            else:
                acc_desired = (UDDSyInterp[j] - UDDSyInterp[j-1]) / dt_substep
            
            # CRITICAL FIX: Resistance forces are NEGATIVE (oppose motion)
            f_tr_desired = (var.rotationalInertiaCoeff * var.vehicleMass * acc_desired + 
                            func.fRoll(UDDSyInterp[j]) + 
                            func.fAerodynamic(UDDSyInterp[j]))
            
            required_power = func.powerFromForce(abs(f_tr_desired), UDDSyInterp[j])
            wheel_angular_vel = func.angularVelocity(UDDSyInterp[j], 1)
            required_wheel_torque = required_power / wheel_angular_vel if wheel_angular_vel > 0.01 else 0
            required_motor_torque = required_wheel_torque / var.gearRatio
            
            # Apply sign based on mode
            if mode == "BRAKING":
                required_motor_torque = -abs(required_motor_torque)
            else:
                required_motor_torque = abs(required_motor_torque)
            
            # Calculate actual achievable current and torque
            motor_current = func.currentFromTorqueAndSpeed(
                motor_angular_vel, required_motor_torque, V_motor
            )
            motor_torque = motor_current * var.motorConstant
            
            # Check for NaN/Inf
            if np.isnan(motor_torque) or np.isinf(motor_torque):
                print(f"ERROR at i={i}, j={j}: motor_torque is {motor_torque}")
                print(f"  motor_current={motor_current}, V_motor={V_motor}")
                print(f"  required_motor_torque={required_motor_torque}")
                motor_torque = 0
                motor_current = 0
        # CRITICAL FIX: Update vehicle dynamics at EACH substep!
        actual_speed, actual_accel = UDDSVehicleSimulation.update_vehicle_dynamics(
            motor_torque, dt_substep
        )
        # Update battery for this substep
        soc, pack_voltage, _ = UDDSBatteryPack.update_battery(motor_current, dt_substep)
        total_torque_impulse += motor_torque * dt_substep
        # Store the final values from this substep
        UDDSVehicleSimulation.motor_current = motor_current
        UDDSVehicleSimulation.motor_torque = motor_torque
        UDDSVehicleSimulation.motor_voltage = V_motor
        # Check for runaway
        if UDDSVehicleSimulation.actual_speed > 100:  # 100 m/s = 360 km/h, clearly wrong
            print(f"\nERROR: Speed runaway detected at i={i}, j={j}")
            print(f"  Speed={UDDSVehicleSimulation.actual_speed:.2f} m/s")
            print(f"  motor_torque={motor_torque:.2f} Nm")
            print(f"  mode={mode}, V_motor={V_motor:.2f} V")
            raise ValueError("Simulation diverged - speed runaway") 
    # Debug output for this major time step
    if DEBUG and i % 5 == 0:
        print(f"\n  Substep summary: {substep_count}")
        print(f"  Total torque impulse: {total_torque_impulse:.2f} Nm·s")
        print(f"  Final speed: {UDDSVehicleSimulation.actual_speed:.2f} m/s")
        print(f"  Final acceleration: {UDDSVehicleSimulation.actual_acceleration:.3f} m/s²")
        print(f"  Speed error: {desired_speed - UDDSVehicleSimulation.actual_speed:.2f} m/s")
        print(f"  Battery SOC: {soc:.1f}%")
    # Save results for this major time step
    UDDSresults.append({
        'time': UDDSTime[i],
        'desired_speed': desired_speed,
        'actual_speed': UDDSVehicleSimulation.actual_speed,
        'actual_acceleration': UDDSVehicleSimulation.actual_acceleration,
        'motor_torque': UDDSVehicleSimulation.motor_torque,
        'motor_current': UDDSVehicleSimulation.motor_current,
        'motor_voltage': UDDSVehicleSimulation.motor_voltage,
        'battery_voltage': pack_voltage,
        'soc': soc,
        'V_ref': V_ref,
        'mode': mode
    })

#plot results
UDDSresults_df = pd.DataFrame(UDDSresults)
plt.figure(figsize=(12, 8))
plt.subplot(2, 2, 1)
plt.plot(UDDSresults_df['time'], UDDSresults_df['actual_speed'], label='Actual Speed', color='green', linewidth=2)
plt.plot(UDDSresults_df['time'], UDDSresults_df['desired_speed'], label='Desired Speed', color='orange', linestyle='--', linewidth=2)
plt.title('Vehicle Speed vs Time (UDDS)')
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 2)
plt.plot(UDDSresults_df['time'], UDDSresults_df['soc'], label='SOC', color='blue', linewidth=2)
plt.title('Battery State of Charge (UDDS)')
plt.xlabel('Time (s)')
plt.ylabel('SOC (%)')
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 3)
plt.plot(UDDSresults_df['time'], UDDSresults_df['motor_torque'], label='Motor Torque', color='red', linewidth=2)
plt.title('Motor Torque (UDDS)')
plt.xlabel('Time (s)')
plt.ylabel('Torque (Nm)')
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 4)
plt.plot(UDDSresults_df['time'], UDDSresults_df['motor_current'], label='Motor Current', color='purple', linewidth=2)
plt.title('Motor Current (UDDS)')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()  
print(f"\nDistance traveled (UDDS): {np.trapezoid(UDDSresults_df['actual_speed'], UDDSresults_df['time']):.2f} meters")

US06VehicleSimulation = VehicleSimulation()
US06BatteryPack = bc.BatteryPack()
US06data = pd.read_csv('US06.csv')
US06Time = US06data['Time (seconds)'].values
US06Speed = US06data['Speed (mph)'].values / 2.237
US06distance = np.trapezoid(US06Speed, US06Time)
US06results = []
DEBUG = False  # Set to False to disable debug prints
for i in range(max(len(US06Time), 50)):  # Limit to 50 for testing
    desired_speed = US06Speed[i]
    
    if i == 0:
        dt = US06Time[0]
    else:
        dt = US06Time[i] - US06Time[i-1]
    
    # Create interpolated substeps
    lineSpaceRange = np.linspace(US06Time[i - 1] if i > 0 else 0, US06Time[i], 100)
    US06yInterp = np.interp(lineSpaceRange, US06Time, US06Speed)
    dt_substep = dt / len(lineSpaceRange)
    
    if DEBUG and i % 5 == 0:
        print(f"\n{'='*70}")
        print(f"TIME STEP {i}: t={US06Time[i]:.1f}s, desired_speed={desired_speed:.2f} m/s")
        print(f"  Current actual_speed={US06VehicleSimulation.actual_speed:.2f} m/s")
        print(f"  dt={dt:.3f}s, dt_substep={dt_substep:.6f}s, num_substeps={len(lineSpaceRange)}")
    
    # Track cumulative torque application
    total_torque_impulse = 0
    substep_count = {'DRIVING': 0, 'BRAKING': 0, 'COASTING': 0}
    
    for j in range(len(US06yInterp)):
        # PI controller generates voltage reference
        V_ref, US06VehicleSimulation.integral = func.pi_controller(
            US06yInterp[j], US06VehicleSimulation.actual_speed, US06VehicleSimulation.integral, dt_substep
        )
        
        speed_error = US06yInterp[j] - US06VehicleSimulation.actual_speed
        
        # Get battery voltage
        V_batt = US06BatteryPack.update_battery_voltage()
        
        # Determine mode and get motor voltage
        if speed_error > 0.1:
            mode = "DRIVING"
            V_motor, V_arm = func.buckConverter(US06BatteryPack.soc, V_batt, V_ref)
        elif speed_error < -0.1:
            mode = "BRAKING"
            V_motor, V_arm = func.regenerativeBrakeConverter(
                US06BatteryPack.soc, V_batt, abs(V_ref)
            )
        else:
            mode = "COASTING"
            V_motor = 0
            V_arm = 0
        substep_count[mode] += 1
        # CRITICAL FIX: If V_motor is 0, we cannot produce torque
        if abs(V_motor) < 0.1:
            motor_current = 0
            motor_torque = 0
        else:   
            motor_angular_vel = func.angularVelocity(US06VehicleSimulation.actual_speed, var.gearRatio)
            
            # Calculate desired acceleration for THIS substep
            if j == 0:
                acc_desired = 0
            else:
                acc_desired = (US06yInterp[j] - US06yInterp[j-1]) / dt_substep
            
            # CRITICAL FIX: Resistance forces are NEGATIVE (oppose motion)
            f_tr_desired = (var.rotationalInertiaCoeff * var.vehicleMass * acc_desired + 
                            func.fRoll(US06yInterp[j]) + 
                            func.fAerodynamic(US06yInterp[j]))
            
            required_power = func.powerFromForce(abs(f_tr_desired), US06yInterp[j])
            wheel_angular_vel = func.angularVelocity(US06yInterp[j], 1)
            required_wheel_torque = required_power / wheel_angular_vel if wheel_angular_vel > 0.01 else 0
            required_motor_torque = required_wheel_torque / var.gearRatio
            
            # Apply sign based on mode
            if mode == "BRAKING":
                required_motor_torque = -abs(required_motor_torque)
            else:
                required_motor_torque = abs(required_motor_torque)
            
            # Calculate actual achievable current and torque
            motor_current = func.currentFromTorqueAndSpeed(
                motor_angular_vel, required_motor_torque, V_motor
            )
            motor_torque = motor_current * var.motorConstant
            
            # Check for NaN/Inf
            if np.isnan(motor_torque) or np.isinf(motor_torque):
                print(f"ERROR at i={i}, j={j}: motor_torque is {motor_torque}")
                print(f"  motor_current={motor_current}, V_motor={V_motor}")
                print(f"  required_motor_torque={required_motor_torque}")
                motor_torque = 0
                motor_current = 0
        # CRITICAL FIX: Update vehicle dynamics at EACH substep!
        actual_speed, actual_accel = US06VehicleSimulation.update_vehicle_dynamics(
            motor_torque, dt_substep
        )
        # Update battery for this substep
        soc, pack_voltage, _ = US06BatteryPack.update_battery(motor_current, dt_substep)
        total_torque_impulse += motor_torque * dt_substep
        # Store the final values from this substep
        US06VehicleSimulation.motor_current = motor_current
        US06VehicleSimulation.motor_torque = motor_torque
        US06VehicleSimulation.motor_voltage = V_motor
        # Check for runaway
        if US06VehicleSimulation.actual_speed > 100:  # 100 m/s = 360 km/h, clearly wrong
            print(f"\nERROR: Speed runaway detected at i={i}, j={j}")
            print(f"  Speed={US06VehicleSimulation.actual_speed:.2f} m/s")
            print(f"  motor_torque={motor_torque:.2f} Nm")
            print(f"  mode={mode}, V_motor={V_motor:.2f} V")
            raise ValueError("Simulation diverged - speed runaway") 
    # Debug output for this major time step
    if DEBUG and i % 5 == 0:
        print(f"\n  Substep summary: {substep_count}")
        print(f"  Total torque impulse: {total_torque_impulse:.2f} Nm·s")
        print(f"  Final speed: {US06VehicleSimulation.actual_speed:.2f} m/s")
        print(f"  Final acceleration: {US06VehicleSimulation.actual_acceleration:.3f} m/s²")
        print(f"  Speed error: {desired_speed - US06VehicleSimulation.actual_speed:.2f} m/s")
        print(f"  Battery SOC: {soc:.1f}%")
    # Save results for this major time step
    US06results.append({
        'time': US06Time[i],
        'desired_speed': desired_speed,
        'actual_speed': US06VehicleSimulation.actual_speed,
        'actual_acceleration': US06VehicleSimulation.actual_acceleration,
        'motor_torque': US06VehicleSimulation.motor_torque,
        'motor_current': US06VehicleSimulation.motor_current,
        'motor_voltage': US06VehicleSimulation.motor_voltage,
        'battery_voltage': pack_voltage,
        'soc': soc,
        'V_ref': V_ref,
        'mode': mode
    })  
#plot results
US06results_df = pd.DataFrame(US06results)
plt.figure(figsize=(12, 8))
plt.subplot(2, 2, 1)
plt.plot(US06results_df['time'], US06results_df['actual_speed'], label='Actual Speed', color='green', linewidth=2)
plt.plot(US06results_df['time'], US06results_df['desired_speed'], label='Desired Speed', color='orange', linestyle='--', linewidth=2)
plt.title('Vehicle Speed vs Time (US06)')
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.legend()
plt.grid(True)      

plt.subplot(2, 2, 2)
plt.plot(US06results_df['time'], US06results_df['soc'], label='SOC', color='blue', linewidth=2)
plt.title('Battery State of Charge (US06)')
plt.xlabel('Time (s)')
plt.ylabel('SOC (%)')
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 3)
plt.plot(US06results_df['time'], US06results_df['motor_torque'], label='Motor Torque', color='red', linewidth=2)
plt.title('Motor Torque (US06)')
plt.xlabel('Time (s)')
plt.ylabel('Torque (Nm)')
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 4)
plt.plot(US06results_df['time'], US06results_df['motor_current'], label='Motor Current', color='purple', linewidth=2)
plt.title('Motor Current (US06)')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()
plt.grid(True)  
plt.tight_layout()
plt.show()  
print(f"\nDistance traveled (US06): {np.trapezoid(US06results_df['actual_speed'], US06results_df['time']):.2f} meters")
