import numpy as np
import scipy as sp
import variable as var
def accelerationCheck(speed, time, previousSpeed, previousTime):
    ## check and calculate acceleration
    acc = 0
    if time == 0:
        acc = (speed - previousSpeed) / (time - previousTime)
    if speed != 0: 
        if time != previousTime:
            if speed != previousSpeed:
                acc = (speed - previousSpeed) / (time - previousTime)
    return acc
def laplaceVariable(omega, w):
    w = w * 1j
    s = omega + w
    return s
def piTransferFunction(kp, ki, s):
    return kp + ki / s

def stateOfCharge(current, time, capacity):
    iDt = sp.integrate.cumtrapz(current, time, initial=0)
    soc = capacity - (iDt + (1 - (var.initialSOC/100)*capacity))
    soc = soc / capacity * 100
    return soc

def fRoll(speed):
    result = 0
    if speed != 0:
        result = var.vehicleMass * var.gravity * (var.rollingResistanceCoeff + var.velocityDependentRollingCoeff * speed**2)
    
    return result

def fAerodynamic(speed):
    result = 0
    if speed != 0:
        result = 0.5 * var.airDensity * var.frontalArea * var.dragCoeff * speed**2
   
    return result   

def fgxt():
    ## beta is 0 for ever so that means the fgxt function will always return 0
    return 0

def powerFromForce(force, speed):
    return force * speed

def powerFromTorque(torque, angularVelocity):
    return torque * angularVelocity

def powerFromVoltageCurrent(voltage, current):
    return voltage * current


def angularVelocity(speed, gearRatio):
    result = 0 
    if gearRatio == 0 or gearRatio is None or gearRatio == np.nan:
        result = speed / var.wheelRadius
    else:
        result = (speed / var.wheelRadius) * gearRatio
    return result

def motorRPM(angularVelocity):
    return (angularVelocity / var.wheelRadius) * (60 / (2 * np.pi)) * var.gearRatio

def pMotor(pM,ptrMax):
    return max(pM, ptrMax)

def vocLookup(soc):
    soc = np.round(soc)
    result = 0
    match soc:
        case 100:
            result = 6.37
        case 90:
            result = 6.31
        case 80:
            result = 6.25
        case 70:
            result = 6.19
        case 60:
            result = 6.12
        case 50:
            result = 6.05
        case 40:
            result = 5.98
        case 30:
            result = 5.91
        case 20:
            result = 5.83
        case 10:
            result = 5.75
        case _:
            result = 0
            print("SOC value out of range for OCV lookup.")
    return result

def current():
    pass

def buckConverter(soc, vBatt, vRef):
    vMotor = 0
    if soc > 20:
        if vRef > 0:  
            if vRef >= vBatt:
                vMotor = vBatt  
            else:
                vMotor = vRef   
        else:
            vMotor = 0  
    else:
        vMotor = 0 
    vArm = vMotor
    return vMotor, vArm

def wheelOmega(power):
    ## calculate the angular velocity based on current speed and resistive forces
    omega = power / var.wheelRadius
    return omega


def motorTorque(wheelTorque, power):
    ## calculate motor torque from wheel torque and power
    if power == 0:
        return 0
    torque = wheelTorque / var.gearRatio
    return torque


def backEMF(angularVelocity):
    return var.backEmfConstant * angularVelocity  

def analyzeDrivingPoint(angularVelocity, requiredTorque):
   
    Ia = requiredTorque / var.motorConstant
    Ea = var.backEmfConstant * angularVelocity
    Va = Ea + (Ia * var.armatureResistance)
    if Va > var.ratedArmatureVoltage:
        Ia = (var.ratedArmatureVoltage - Ea) / var.armatureResistance
        actualTorque = Ia * var.motorConstant
        Va = var.ratedArmatureVoltage
    else:
        actualTorque = requiredTorque
    
    return Va, Ia, actualTorque
def regenerativeBrakeConverter(soc, vBatt, vRef):
    
    if soc >= 100:  
        vMotor = 0
    else:
        
        vMotor = -min(vRef, vBatt * 0.8)  
    vArm = vMotor
    return vMotor, vArm
def currentFromTorqueAndSpeed(angularVelocity, torque, terminalVoltage):
    """
    Calculate motor current based on torque, speed, and available voltage.
    
    CRITICAL FIX: When terminal voltage is 0, motor cannot produce torque
    regardless of back-EMF conditions.
    """
    # If no voltage applied, no current can flow (except in regen with external circuit)
    if abs(terminalVoltage) < 0.1:  # Essentially zero voltage
        return 0.0
    
    # Required current for torque
    I_torque = torque / var.motorConstant
    
    # Back EMF voltage
    E_a = var.backEmfConstant * angularVelocity
    
    # Available voltage for current flow
    available_voltage = terminalVoltage - E_a
    
    # For motoring (positive torque)
    if I_torque > 0:
        if available_voltage <= 0:
            # Cannot produce positive torque - back EMF too high
            return 0.0
        
        # Maximum possible current given voltage constraint
        max_current = available_voltage / var.armatureResistance
        return min(I_torque, max_current, var.ratedArmatureCurrent)
    
    # For braking (negative torque/current)
    else:
        # In regenerative braking, we need a charging circuit
        # If terminalVoltage = 0, we can't brake electrically
        if abs(terminalVoltage) < 1.0:
            return 0.0
        
        # Available voltage must be negative for regen
        if available_voltage >= 0:
            return 0.0
            
        max_current = available_voltage / var.armatureResistance
        return max(I_torque, max_current, -var.ratedArmatureCurrent)
    
def pi_controller(desired_speed, actual_speed, integral, dt):
    """PI controller generates V_ref for power electronics"""
    error = desired_speed - actual_speed
    integral += error * dt
    
    V_ref = var.kpSpeed * error + var.kiSpeed * integral
   
    V_ref = np.clip(V_ref, 0, var.ratedArmatureVoltage)
    
    return V_ref, integral
print("Functions module loaded.")