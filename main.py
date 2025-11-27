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
        self.mechanical_brake_force = 0.0  # Track mechanical braking
        
    def update_vehicle_dynamics(self, motor_torque, mechanical_brake_torque, dt):
        """Update actual vehicle speed based on motor torque and mechanical braking"""
        # Motor traction
        wheel_torque = motor_torque * var.gearRatio * 0.95
        traction_force = wheel_torque / var.wheelRadius
        
        # Mechanical braking (friction brakes)
        brake_force = -mechanical_brake_torque / var.wheelRadius  # Negative because it opposes motion

        # Resistance forces OPPOSE motion (negative)
        roll_force = -func.fRoll(self.actual_speed)
        aero_force = -func.fAerodynamic(self.actual_speed)
        total_resistance = roll_force + aero_force
        
        net_force = traction_force + brake_force + total_resistance
        
        # Check for numerical issues
        if np.isnan(net_force) or np.isinf(net_force):
            print(f"ERROR: net_force is {net_force}")
            return self.actual_speed, 0.0
        
        if self.actual_speed == 0 and net_force <= 0:
            self.actual_acceleration = 0
        else:
            self.actual_acceleration = net_force / (var.vehicleMass * var.rotationalInertiaCoeff)
        
        # Update speed
        new_speed = self.actual_speed + self.actual_acceleration * dt
        self.actual_speed = max(0, new_speed)
        
        self.mechanical_brake_force = brake_force
        
        return self.actual_speed, self.actual_acceleration

def improved_pi_controller(desired_speed, actual_speed, integral, dt, max_integral=50):
    """
    Improved PI controller with anti-windup
    """
    error = desired_speed - actual_speed
    
    # Anti-windup: only integrate if not saturated
    integral_temp = integral + error * dt
    
    # Calculate tentative output
    V_ref = var.kpSpeed * error + var.kiSpeed * integral_temp
    
    # If output would saturate, don't update integral (anti-windup)
    if V_ref > var.ratedArmatureVoltage:
        V_ref = var.ratedArmatureVoltage
    elif V_ref < 0:
        V_ref = 0
    else:
        integral = integral_temp
        integral = np.clip(integral, -max_integral, max_integral)
    
    return V_ref, integral

def simulate_drive_cycle(data_file, cycle_name, num_substeps=100, debug=False):
    """Run simulation for a drive cycle"""
    
    vehicle = VehicleSimulation()
    batteryPack = bc.BatteryPack()
    
    # Load data
    data = pd.read_csv(data_file)
    Time = data['Time (seconds)'].values
    Speed = data['Speed (mph)'].values / 2.237  # Convert to m/s
    distance_desired = np.trapezoid(Speed, Time)
    
    results = []
    
    print(f"\n{'='*70}")
    print(f"Starting {cycle_name} Simulation")
    print(f"{'='*70}")
    print(f"Duration: {Time[-1]:.1f} seconds")
    print(f"Max speed: {Speed.max():.2f} m/s ({Speed.max()*2.237:.1f} mph)")
    print(f"Number of substeps: {num_substeps} per major timestep")
    
    for i in range(len(Time)):
        desired_speed = Speed[i]
        
        if i == 0:
            dt = Time[0]
        else:
            dt = Time[i] - Time[i-1]
        
        # Create interpolated substeps
        lineSpaceRange = np.linspace(Time[i - 1] if i > 0 else 0, Time[i], num_substeps)
        yInterp = np.interp(lineSpaceRange, Time, Speed)
        dt_substep = dt / len(lineSpaceRange)
        
        if debug and i % 100 == 0:
            print(f"Time {Time[i]:.1f}s: desired={desired_speed:.2f}, actual={vehicle.actual_speed:.2f} m/s")
        
        substep_count = {'DRIVING': 0, 'BRAKING': 0, 'COASTING': 0, 'MECH_BRAKE': 0}
        
        for j in range(len(yInterp)):
            # PI controller generates voltage reference
            V_ref, vehicle.integral = improved_pi_controller(
                yInterp[j], vehicle.actual_speed, vehicle.integral, dt_substep
            )
            
            speed_error = yInterp[j] - vehicle.actual_speed
            V_batt = batteryPack.update_battery_voltage()
            
            # Calculate required deceleration
            if j == 0 or dt_substep < 1e-9:
                acc_desired = 0
            else:
                acc_desired = (yInterp[j] - yInterp[j-1]) / dt_substep
            
            # CRITICAL: Detect if we need aggressive braking
            # If deceleration requirement is > 3 m/s² or speed error is large negative
            need_aggressive_brake = (acc_desired < -3.0) or (speed_error < -2.0)
            
            # Determine mode
            mechanical_brake_torque = 0
            
            if speed_error > 0.1:
                mode = "DRIVING"
                V_motor, V_arm = func.buckConverter(batteryPack.soc, V_batt, V_ref)
                
            elif need_aggressive_brake:
                # Use BOTH regenerative AND mechanical braking
                mode = "MECH_BRAKE"
                V_motor, V_arm = func.regenerativeBrakeConverter(
                    batteryPack.soc, V_batt, abs(V_ref)
                )
                
                # Add mechanical braking force
                # Calculate maximum safe deceleration (typically -8 m/s² for cars)
                max_decel = -8.0  # m/s²
                required_decel = max(acc_desired, max_decel)  # Limit to safe deceleration
                
                # Calculate total braking force needed
                total_brake_force = abs(var.vehicleMass * var.rotationalInertiaCoeff * required_decel)
                
                # Subtract what regen can provide, rest is mechanical
                # Estimate regen force (will be calculated properly later)
                motor_angular_vel = func.angularVelocity(vehicle.actual_speed, var.gearRatio)
                regen_current_estimate = min(100, abs(V_motor) / var.armatureResistance)  # Rough estimate
                regen_torque_estimate = regen_current_estimate * var.motorConstant
                regen_force_estimate = regen_torque_estimate * var.gearRatio * 0.95 / var.wheelRadius
                
                # Mechanical brake provides the rest
                mechanical_brake_force = max(0, total_brake_force - regen_force_estimate)
                mechanical_brake_torque = mechanical_brake_force * var.wheelRadius
                
            elif speed_error < -0.1:
                mode = "BRAKING"
                V_motor, V_arm = func.regenerativeBrakeConverter(
                    batteryPack.soc, V_batt, abs(V_ref)
                )
                
            else:
                mode = "COASTING"
                V_motor = 0
                V_arm = 0
            
            # Reset integral when stopped
            if vehicle.actual_speed < 0.01 and yInterp[j] < 0.01:
                vehicle.integral = 0
            
            substep_count[mode] += 1
            
            # Calculate motor torque
            if abs(V_motor) < 0.1:
                motor_current = 0
                motor_torque = 0
            else:
                motor_angular_vel = func.angularVelocity(vehicle.actual_speed, var.gearRatio)
                
                # Calculate forces needed
                inertial_force = var.rotationalInertiaCoeff * var.vehicleMass * acc_desired
                resistance_force = func.fRoll(yInterp[j]) + func.fAerodynamic(yInterp[j])
                
                # For mechanical braking mode, only use motor for regen assist
                if mode == "MECH_BRAKE":
                    # Motor just does regen, mechanical does the rest
                    f_traction_needed = inertial_force + resistance_force
                    required_motor_force = min(0, f_traction_needed)  # Only negative (braking)
                else:
                    # Normal control
                    f_traction_needed = inertial_force + resistance_force
                    required_motor_force = f_traction_needed
                
                # Convert to motor torque
                required_wheel_torque = required_motor_force * var.wheelRadius
                required_motor_torque = required_wheel_torque / (var.gearRatio * 0.95)
                
                # Apply mode-based sign
                if mode in ["BRAKING", "MECH_BRAKE"]:
                    required_motor_torque = -abs(required_motor_torque)
                
                # Calculate actual achievable current and torque
                motor_current = func.currentFromTorqueAndSpeed(
                    motor_angular_vel, required_motor_torque, V_motor
                )
                motor_torque = motor_current * var.motorConstant
                
                # Safety checks
                if np.isnan(motor_torque) or np.isinf(motor_torque):
                    motor_torque = 0
                    motor_current = 0
            
            # Update vehicle dynamics with BOTH motor and mechanical braking
            actual_speed, actual_accel = vehicle.update_vehicle_dynamics(
                motor_torque, mechanical_brake_torque, dt_substep
            )
            
            # Update battery (mechanical braking doesn't affect battery)
            soc, pack_voltage, _ = batteryPack.update_battery(motor_current, dt_substep)
            
            # Store values
            vehicle.motor_current = motor_current
            vehicle.motor_torque = motor_torque
            vehicle.motor_voltage = V_motor
            
            # Safety check for runaway
            if vehicle.actual_speed > 100:
                print(f"ERROR: Speed runaway at t={Time[i]:.1f}s")
                raise ValueError("Simulation diverged")
        
        # Save results for this major time step
        results.append({
            'time': Time[i],
            'desired_speed': desired_speed,
            'actual_speed': vehicle.actual_speed,
            'actual_acceleration': vehicle.actual_acceleration,
            'motor_torque': vehicle.motor_torque,
            'motor_current': vehicle.motor_current,
            'motor_voltage': vehicle.motor_voltage,
            'battery_voltage': pack_voltage,
            'soc': soc,
            'V_ref': V_ref,
            'mode': mode,
            'speed_error': desired_speed - vehicle.actual_speed,
            'mechanical_brake_force': vehicle.mechanical_brake_force
        })
    
    results_df = pd.DataFrame(results)
    distance_actual = np.trapezoid(results_df['actual_speed'], results_df['time'])
    
    # Performance metrics
    print(f"\n{cycle_name} Simulation Complete!")
    print(f"{'='*70}")
    print(f"Final speed: {vehicle.actual_speed:.2f} m/s")
    print(f"Final SOC: {soc:.1f}%")
    print(f"Distance (desired): {distance_desired:.2f} m")
    print(f"Distance (actual): {distance_actual:.2f} m")
    print(f"Distance error: {abs(distance_actual - distance_desired):.2f} m ({100*abs(distance_actual - distance_desired)/distance_desired:.2f}%)")
    
    # Braking statistics
    mech_brake_used = (results_df['mechanical_brake_force'] < -10).sum()
    print(f"Mechanical braking used: {mech_brake_used} times ({100*mech_brake_used/len(results_df):.1f}% of time)")
    
    # Speed tracking analysis
    speed_error = results_df['speed_error']
    print(f"\nSpeed Tracking Performance:")
    print(f"  Mean error: {speed_error.mean():.3f} m/s")
    print(f"  RMS error: {np.sqrt((speed_error**2).mean()):.3f} m/s")
    print(f"  Max error: {speed_error.abs().max():.3f} m/s (at t={results_df.loc[speed_error.abs().idxmax(), 'time']:.1f}s)")
    print(f"  % of time within ±1 m/s: {100 * (speed_error.abs() <= 1.0).sum() / len(speed_error):.1f}%")
    print(f"  % of time within ±2 m/s: {100 * (speed_error.abs() <= 2.0).sum() / len(speed_error):.1f}%")
    
    return results_df

def plot_cycle_results(df, cycle_name):
    """Create separate plot windows for each cycle"""
    
    # Figure 1: Speed
    plt.figure(figsize=(12, 6))
    plt.plot(df['time'], df['actual_speed'], 'g-', linewidth=2, label='Actual Speed')
    plt.plot(df['time'], df['desired_speed'], 'r--', linewidth=1.5, label='Desired Speed', alpha=0.7)
    plt.title(f'{cycle_name} - Vehicle Speed vs Time', fontsize=14, fontweight='bold')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Speed (m/s)', fontsize=12)
    plt.legend(fontsize=11, loc='upper right')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Figure 2: Battery SOC
    plt.figure(figsize=(10, 6))
    plt.plot(df['time'], df['soc'], 'b-', linewidth=2)
    plt.title(f'{cycle_name} - Battery State of Charge', fontsize=14, fontweight='bold')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('SOC (%)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Figure 3: Motor Torque
    plt.figure(figsize=(10, 6))
    plt.plot(df['time'], df['motor_torque'], 'r-', linewidth=2)
    plt.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    plt.title(f'{cycle_name} - Motor Torque', fontsize=14, fontweight='bold')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Torque (Nm)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Figure 4: Braking Analysis
    plt.figure(figsize=(12, 6))
    ax1 = plt.gca()
    ax1.plot(df['time'], df['motor_current'], 'purple', linewidth=2, label='Motor Current')
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Motor Current (A)', fontsize=12, color='purple')
    ax1.tick_params(axis='y', labelcolor='purple')
    ax1.grid(True, alpha=0.3)
    
    # Add mechanical brake force on secondary axis
    ax2 = ax1.twinx()
    ax2.plot(df['time'], df['mechanical_brake_force'], 'orange', linewidth=2, label='Mech Brake Force', alpha=0.7)
    ax2.set_ylabel('Mechanical Brake Force (N)', fontsize=12, color='orange')
    ax2.tick_params(axis='y', labelcolor='orange')
    
    plt.title(f'{cycle_name} - Motor Current & Mechanical Braking', fontsize=14, fontweight='bold')
    
    # Combine legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    plt.tight_layout()
    
    # Figure 5: Speed Error
    plt.figure(figsize=(12, 6))
    plt.plot(df['time'], df['speed_error'], 'm-', linewidth=1.5, alpha=0.7)
    plt.axhline(y=0, color='k', linestyle='-', linewidth=1)
    plt.axhline(y=2, color='r', linestyle='--', linewidth=0.8, alpha=0.5, label='±2 m/s')
    plt.axhline(y=-2, color='r', linestyle='--', linewidth=0.8, alpha=0.5)
    plt.axhline(y=1, color='orange', linestyle='--', linewidth=0.8, alpha=0.5, label='±1 m/s')
    plt.axhline(y=-1, color='orange', linestyle='--', linewidth=0.8, alpha=0.5)
    plt.title(f'{cycle_name} - Speed Tracking Error', fontsize=14, fontweight='bold')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Error (Desired - Actual) (m/s)', fontsize=12)
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.ylim(-5, 5)
    plt.tight_layout()
    
    # Figure 6: Battery Voltage
    plt.figure(figsize=(10, 6))
    plt.plot(df['time'], df['battery_voltage'], 'orange', linewidth=2)
    plt.title(f'{cycle_name} - Battery Voltage', fontsize=14, fontweight='bold')
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Voltage (V)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

# Run simulations
print("\n" + "="*70)
print("ELECTRIC VEHICLE SIMULATION - WITH MECHANICAL BRAKING")
print("="*70)

HWresults_df = simulate_drive_cycle('HWFET.csv', 'HWFET', num_substeps=100, debug=False)
UDDSresults_df = simulate_drive_cycle('UDDS.csv', 'UDDS', num_substeps=100, debug=False)
US06results_df = simulate_drive_cycle('US06.csv', 'US06', num_substeps=100, debug=False)

print("\n" + "="*70)
print("Generating plots...")
print("="*70)

# Create all plots in separate windows
plot_cycle_results(HWresults_df, 'HWFET')
plot_cycle_results(UDDSresults_df, 'UDDS')
plot_cycle_results(US06results_df, 'US06')

print("\nAll plots generated! Close plot windows to end program.")
print("="*70)

plt.show()

print("\n" + "="*70)
print("SIMULATION SUMMARY")
print("="*70)
print(f"HWFET Distance: {np.trapezoid(HWresults_df['actual_speed'], HWresults_df['time']):.2f} m")
print(f"UDDS Distance:  {np.trapezoid(UDDSresults_df['actual_speed'], UDDSresults_df['time']):.2f} m")
print(f"US06 Distance:  {np.trapezoid(US06results_df['actual_speed'], US06results_df['time']):.2f} m")
print("="*70)