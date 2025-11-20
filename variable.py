"""
Electric Vehicle System Parameters
EECS 4646 - Final Project
"""

# =============================================================================
# BATTERY PACK (Subsystem #3)
# =============================================================================

# Battery Specifications
batteryVoltage = 6  # V
batteryCapacity = 225  # AH (Ampere-hours)

# Battery Equivalent Circuit Parameters
r1Battery = 0.01726  # Ω
r2Battery = 0.0104  # Ω
c1Battery = 30991.735  # F
c2Battery = 8476744.18  # F
voltageSource = 5.75  # V

# Note: SOC vs OCV table would be implemented separately

# =============================================================================
# VEHICLE DYNAMICS
# =============================================================================

vehicleMass = 1000  # kg
wheelRadius = 0.28  # m
rotationalInertiaCoeff = 1.1  # km (dimensionless)
gearRatio = 10  # GR (dimensionless)
gravity = 9.81  # m/s²

# Rolling Resistance Coefficients
c0Rolling = 0.009  # sec²/m²
c1Rolling = 1.7e-6  # sec²/m²

# Aerodynamic Parameters
dragCoeff = 0.2  # CD (dimensionless)
frontalArea = 2  # m²
airDensity = 1.17  # kg/m³

# Initial Conditions
beta = 0  # road grade (radians)
initialVelocity = 0  # m/s

# =============================================================================
# MOTOR MODEL
# =============================================================================

# Motor Specifications
ratedArmatureVoltage = 320  # V
ratedArmatureCurrent = 600  # A

# Motor Electrical Parameters
armatureResistance = 0.5  # Ω
armatureInductance = 0.008  # H (8 mH converted to H)
backEmfConstant = 0.2  # V·sec/rad
motorConstant = 0.2

# Motor Speed Control (PID Parameters)
kpSpeed = 300  # Proportional gain
kiSpeed = 1.5  # Integral gain

# =============================================================================
# DERIVED PARAMETERS (for convenience)
# =============================================================================

# Convert battery capacity to Coulombs
batteryCapacityCoulombs = batteryCapacity * 3600  # A·s (Coulombs)

# Vehicle characteristic parameters (for resistance calculations)
rollingResistanceCoeff = c0Rolling  # Primary rolling resistance coefficient
velocityDependentRollingCoeff = c1Rolling  # Velocity-dependent rolling resistance

print("Variable module loaded.")