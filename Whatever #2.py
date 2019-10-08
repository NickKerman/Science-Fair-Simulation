
import math
from mpmath import *
import numpy as np

h = 59
MMA = 28.9644
gs = 9.80665
Rugc = 8314.32
temp_lapse_ratek = -0.0065
T = 15 + (temp_lapse_ratek * h)
static_pressure = 101325
standard_tempk = 288.15
layer_heightm = 0
default_layer_density = 1.2250
pa = standard_tempk / ((standard_tempk + (temp_lapse_ratek * (h - layer_heightm))))
pb = (gs * MMA) / (Rugc * temp_lapse_ratek)
localPressure = static_pressure * math.pow(pa, pb)
localAirDensity = default_layer_density * math.pow(pa, (1+pb))

print localPressure

stageOneStartThrust = 34354772
stageOneMaxThrust = 40064144
stageOneOriginalStartLocalPressure = 165.075263457
stageOneOriginalMaxLocalPressure = 100610

ThrustN = stageOneMaxThrust - (((stageOneMaxThrust - stageOneStartThrust) /
                                  (stageOneOriginalMaxLocalPressure - stageOneOriginalStartLocalPressure))
                                      * localPressure)

print ThrustN