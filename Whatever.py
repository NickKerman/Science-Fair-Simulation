import math
from mpmath import *
import numpy as np

h = 43900
MMA = 28.9644
gs = 9.80665
Rugc = 8314.32
T = -44.5 + 0.0028 * (h - 32000)
static_pressure = 868.02
standard_tempk = 228.65
layer_heightm = 32000
temp_lapse_ratek = 0.0028
T = -56.5 + 0.0028 * (h - layer_heightm)
default_layer_density = 0.01322
pa = standard_tempk / ((standard_tempk + (temp_lapse_ratek * (h - layer_heightm))))
pb = ((gs * MMA) / (Rugc * temp_lapse_ratek))
localPressure = static_pressure * math.pow(pa, pb)
localAirDensity = default_layer_density * math.pow(pa, (1 + pb))

print localPressure

stageOneStartThrust = 34354772
stageOneMaxThrust = 40064144
stageOneOriginalStartLocalPressure = 165.075263457
stageOneOriginalMaxLocalPressure = 100610

ThrustN = stageOneMaxThrust - (((stageOneMaxThrust - stageOneStartThrust) /
                                (stageOneOriginalMaxLocalPressure - stageOneOriginalStartLocalPressure))
                                 * localPressure)

print ThrustN