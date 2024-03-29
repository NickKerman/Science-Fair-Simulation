import math
from mpmath import *
import numpy as np
import matplotlib.pyplot as plt

import VehicleSpecs

#v = VehicleSpecs.SaturnV()
v = VehicleSpecs.SLSBlock1B()
#v = VehicleSpecs.SLSBlock2()

planetRadius = 6378137  # Radius of Earth in meters
EarthMass = 5973600000000000000000000
ExcessEscapeVelocity = None # My Independent Variable (in m/s)

t = 0                 # time in seconds
TimeStep = 0.01          # Timefactor (seconds per cycle) lower=more accurate
CounterLimit = 10     # How many iterations the counter will go up to before triggering and reseting
Counter = 0           # counts iterations up until a given number then resets
#simRunTime = 18000    # run time
simRunTime = 7200    # run time

StageNo = 1
localPressure = 0     # pressure of local atmosphere in Pascals
T = 15                # temperature of local atmosphere in degrees C
h = 0                 # height in meters from sea level
C = 120               # Sutherland's constant for air
To = 18               # Reference temperature for viscosity equation
u = 0                 # Viscocity of the air in centipoise
yg = 1.40             # ratio of specific heat of at constant pressure to that at constant volume (is constant)
c = 340               # local speed of sound in m/s
MachNo = 0
M = v.getLaunchMass(StageNo)
ThrustN = 0           # Thrust generated by currently used thrusters in newtons
V = 0                 # current velocity m/s
EscV = 11186          # Escape Velocity of Earth in m/s
forceOfDrag = 0       # Drag force in Newton/seconds
x = planetRadius      # x coordinate in meters (0,0 is center of earth)
y = 0                 # y coordinate in meters
VelocityX = 0         # Velocity along the x axis
VelocityY = 0         # Velocity along the y axis
PolarCoordMag = planetRadius # distance from center (polar coordinates)
PolarCoordDeg = 0     # angle from 0 in degrees (polar coordinates)
xT = 0                # Thrust in x direction
yT = 0                # Thrust in y direction
F = 0                 # net force (after drag)
xF = 0                # force along the x axis
yF = 0                # force along the y axis
GravConst = 0.0000000000667359  #Gravitational constant

gs = 9.80665          # gravity at surface level in m/s^2
GravAng = 180         # direction of gravity's pull from the reference point of the spacecraft
g = 9.80665           # current percieved acceleration due to gravity in m/s
GravForceX = 0        # force of gravity in the x direction in Newtons
GravForceY = 0        # force of gravity in the y direction in Newtons
accel = 0                 # current acceleration in m/s^2 (after gravity)
HeadingDeg = 0        # heading in degrees, 0 is up (also opposite of direction of force) (where you are pointing)
t_str = '0'           # time in seconds in string form for naming structures
gs = 9.80665          # g at surface level in m/s^2
localAirDensity = 0                 # density of local air in kg/m^3
MMA = 28.9644         # molar mass of Earth's air in kg/mol
Rugc = 8314.32        # universal gas constant J/mol/K
altRugc = 286         # alternative universal gas constant
FlightPathDeg = 0     # flight path angle in degrees (where you are going)
GravForce = 0         # Force of gravity in Newtons

# Declare a bunch of empty vectors to store out accumulated data
all_localPressure = []
all_t = []
all_c = []
all_g = []
all_F = []
all_accel = []
all_V = []
all_StageNo = []
all_forceOfDrag = []
all_FlightPathDeg = []
all_forceDragX = []
all_forceDragY = []
all_Ax = []
all_Ay = []
all_GravForce = []
all_GravAng = []
all_GravForceX = []
all_GravForceY = []
all_ThrustN = []
all_forceOfDrag = []
all_VelocityX = []
all_VelocityY = []
all_HeadingDeg = []
all_M = []
all_u = []
all_y = []
all_x = []
all_PolarCoordMag = []
all_PolarCoordDeg = []
all_T = []
all_h = []
all_localAirDensity = []
all_localPressure = []
all_xT = []
all_yT = []
all_dynamicQ = []

stop = 0

while stop == 0:
  t = t + TimeStep
  # atmospheric details
  if (h <= 11000):                               # pressure, density, and temperature functions for under 11000 meters
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
  elif (h <= 20000):                            # pressure, density, and temperature functions for under 20000 meters
    T = -56.5;
    default_layer_density = 0.36391
    static_pressure = 22632.10
    layer_heightm = 11000
    standard_tempk = 216.65
    localPressure = static_pressure * math.exp((-gs * MMA * (h-layer_heightm)) / (Rugc * standard_tempk));
    localAirDensity = default_layer_density * math.exp((-gs * MMA * (h-layer_heightm)) / (Rugc * standard_tempk));
  elif (h <= 32000):                            # pressure, density, and temperature functions for under 32000 meters
    static_pressure = 5474.89
    standard_tempk = 216.65
    temp_lapse_ratek = 0.001
    layer_heightm = 20000;
    T = -56.5 + temp_lapse_ratek * (h-layer_heightm)
    default_layer_density = 0.08803
    pa = standard_tempk / ((standard_tempk+(temp_lapse_ratek*(h-layer_heightm))))
    pb = (gs * MMA) / (Rugc * temp_lapse_ratek)
    localPressure = static_pressure * math.pow(pa, pb)
    localAirDensity = default_layer_density * math.pow(pa, (1+pb))
  elif (h <= 47000):                            # pressure, density, and temperature functions for under 47000 meters
    static_pressure = 868.02
    standard_tempk = 228.65
    layer_heightm = 32000
    temp_lapse_ratek = 0.0028
    T = -56.5 + 0.0028*(h-layer_heightm)
    default_layer_density = 0.01322
    pa = standard_tempk / ((standard_tempk+(temp_lapse_ratek * (h-layer_heightm))))
    pb = ((gs*MMA) / (Rugc*temp_lapse_ratek))
    localPressure = static_pressure * math.pow(pa, pb)
    localAirDensity = default_layer_density * math.pow(pa, (1+pb))
  elif (h <= 51000):                            # pressure, density, and temperature functions for under 51000 meters
    T = -2.5;
    static_pressure = 110.91
    standard_tempk = 270.65
    layer_heightm = 47000
    default_layer_density = 0.00143
    localPressure = static_pressure * math.exp((-gs*MMA*(h-layer_heightm))/(Rugc*(standard_tempk)))
    localAirDensity = default_layer_density * math.exp((-gs*MMA*(h-layer_heightm))/(Rugc*(standard_tempk)))
  elif (h <= 71000):                            # pressure, density, and temperature functions for under 71000 meters
    T = -2.5 + -0.0028*(h-51000)
    static_pressure = 66.94
    temp_lapse_ratek = -0.0028
    standard_tempk = 270.65
    layer_heightm = 51000
    default_layer_density = 0.00086
    pa = standard_tempk / ((standard_tempk + (temp_lapse_ratek * (h - layer_heightm))))
    pb = ((gs * MMA)/(Rugc * temp_lapse_ratek))
    localPressure = static_pressure * math.pow(pa, pb)
    localAirDensity = default_layer_density * math.pow(pa, (1+pb))
  elif (h <= 86000):                            # pressure and temperature functions for under 86000 meters
    T = -58.5 -0.002 * (h-71000)
    static_pressure = 3.96
    temp_lapse_ratek = -0.002
    standard_tempk = 214.65
    layer_heightm = 71000
    default_layer_density = 0.000064
    pa = standard_tempk/((standard_tempk+(temp_lapse_ratek*(h-layer_heightm))))
    pb = ((gs*MMA)/(Rugc*temp_lapse_ratek))
    localPressure = static_pressure * math.pow(pa, pb)
    localAirDensity = default_layer_density * math.pow(pa, (1+pb));
  else:
    localPressure = 0
    localAirDensity = 0

  M, ThrustN, forceOfDrag, StageNo = v.MassAndStageChanger(t, TimeStep, StageNo, localAirDensity, localPressure, V, M, MachNo)

  # The Physics Department

  thrustWeightRatio = ThrustN / (M * g)

  try:
    c = math.sqrt(yg * altRugc * (T+273.15))                             # Local speed of sound (m/s)
    MachNo = V/c
    #u = 0.01827 * ((To + 273.15 + C) / (T+ 273.15 + C)) * math.pow(T / To, 3/2) # Viscocity from temperature and sutherland's constant in centipoise
    forceDragX = forceOfDrag * math.cos(math.radians(FlightPathDeg))     # Force of drag in newtons along x
    forceDragY = forceOfDrag * math.sin(math.radians(FlightPathDeg))     # Force of drag in newtons along y
    xT = ThrustN * math.cos(math.radians(HeadingDeg))                    # thrust along the x axis
    yT = ThrustN * math.sin(math.radians(HeadingDeg))                    # thrust along the y axis
    g = gs * math.pow(planetRadius/PolarCoordMag, 2)                     # acceleration due to gravity
    GravAng = PolarCoordDeg                                              # Direction of acceleration due to gravity from spacecraft's reference point in degrees
    GravForce = g*M                                                      # Force of gravity in Newtons
    GravForceX = 0 - (GravForce * math.cos(math.radians(GravAng)))       # Force due to gravity (in Newtons) in the x direction from g and the negative of the polar position in degrees
    GravForceY = 0 - (GravForce * math.sin(math.radians(GravAng)))       # Force due to gravity (in Newtons) in the y direction from g and the negative of the polar position in degrees
    xF = xT + GravForceX + forceDragX                                    # Total force along x in Newtons
    yF = yT + GravForceY + forceDragY                                    # Total force along y in Newtons
    Ax = xF/M                                                            # Acceleration along x in m/s
    Ay = yF/M                                                            # Acceleration along y in m/s
    accel = math.sqrt(math.pow(Ax, 2) + math.pow(Ay, 2))                 # Acceleration as axes into a vector
    VelocityX = VelocityX + (Ax*TimeStep)                                # speed in x changes due to acceleration
    VelocityY = VelocityY + (Ay*TimeStep)                                # speed in y changes due to acceleration
    V = math.sqrt(math.pow(VelocityX, 2) + math.pow(VelocityY, 2))
    x = x + (VelocityX*TimeStep)                                         # position in x changes due to acceleration along x
    y = y + (VelocityY*TimeStep)                                         # position in y changes due to acceleration along y

    PolarCoordDeg = math.degrees(math.atan2(y, x))
    if PolarCoordDeg < 0:
        PolarCoordDeg = PolarCoordDeg + 360
    PolarCoordMag = math.sqrt(math.pow(x, 2) + math.pow(y, 2))

    h = PolarCoordMag - planetRadius

    velocityAngle = math.degrees(math.atan2(VelocityY, VelocityX))
    if velocityAngle < 0:
        velocityAngle = velocityAngle + 360
    FlightPathDeg = velocityAngle

    dynamicQ = localPressure * math.pow(V , 2) * 0.5
    if (PolarCoordMag == 0):
      stop = t
    if (h < 0):
      stop = t
      #raise Exception
      print ("BOOOOM!  Hit the ground at t=%s" % t)

  except OverflowError:
    print ('Ovewrflow error!')
    stop = t

  #HeadingDeg, kickHeight, kickAngleDeg = v.PitchControl(t, TimeStep, h, FlightPathDeg, HeadingDeg, PolarCoordDeg)
  HeadingDeg, kickAngleDeg, kickHeight = v.PitchControl(t, TimeStep, h, FlightPathDeg, HeadingDeg, PolarCoordDeg, dynamicQ)

  # Accumulate records every 100 itterations
  Counter = (Counter + 1)
  if (Counter == CounterLimit):
    Counter = 0;
    all_t.append(t)                  # These lines record the current values in the one_time structre
    all_y.append(y)
    all_h.append(h)
    all_StageNo.append(StageNo)
    all_localPressure.append(localPressure)
    all_accel.append(accel)
    all_localAirDensity.append(localAirDensity)
    all_Ax.append(Ax)
    all_Ay.append(Ay)
    all_HeadingDeg.append(HeadingDeg)
    all_V.append(V)
    all_VelocityX.append(VelocityX)
    all_VelocityY.append(VelocityY)
    all_x.append(x)
    all_GravForce.append(GravForce)
    all_forceOfDrag.append(forceOfDrag)
    all_FlightPathDeg.append(FlightPathDeg)
    all_GravAng.append(GravAng)
    all_ThrustN.append(ThrustN)
    all_PolarCoordMag.append(PolarCoordMag)
    all_PolarCoordDeg.append(math.radians(PolarCoordDeg))
    all_dynamicQ.append(dynamicQ)
  if (t >= simRunTime):
    stop = 1
print ("Done! Stop value:%s" % stop)

# plot(all_t, all_FlightPathDeg, ";FlightPathDeg;");
# plot(all_t, all_V, ";V;");
# plot(all_t, all_GravAng, ";GravAng;");
# plot(all_t, all_GravForceX, ";GravForceX;", all_t, all_GravForceY, ";GravForceY;");
# plot(all_t, all_forceDragX, ";forceDragX;", all_t, all_forceDragY, ";forceDragY;");
# plot(all_t, all_h, ";Altitude (m);");
# plot(all_t, all_ThrustN, ";Thrust (N);");
# plot(all_t, all_xT, ";X Thrust;", all_t, all_yT, ";Y Thrust;");
# plot(all_t, all_y, ";Y position (m);", all_t, all_x, ";X position (m);");

timeIndexAtBurnout = None
if v.timeAtBurnout is not None:
  timeIndexAtBurnout = int((v.timeAtBurnout/TimeStep)/CounterLimit) - 1

print ('position at burnout was (%s,%s) meters' % (all_x[timeIndexAtBurnout], all_y[timeIndexAtBurnout]))
print ('velocity at burnout was %s m/s' % all_V[timeIndexAtBurnout])
print ('Altitude at burnout was %s meters' % all_h[timeIndexAtBurnout])
EscV = math.sqrt((2*GravConst*EarthMass)/all_PolarCoordMag[timeIndexAtBurnout])
print ('escape velocity at %s meters is = %s m/s' % (all_h[timeIndexAtBurnout], EscV))
ExcessEscapeVelocity = math.sqrt((math.pow(V,2))-(math.pow(EscV,2)))
print ('Excess escape velocity = %s' %ExcessEscapeVelocity)


# X and Y over time

graph_dpi = 100

plt.figure()
fig, ax1 = plt.subplots()
fig.set_size_inches(11, 5)

ax1.plot(all_t, all_y, 'b-')
ax1.set_xlabel('time (s)')
ax1.set_ylabel('Y position (m)', color='b')
ax1.tick_params('y', colors='b')

ax2 = ax1.twinx()
ax2.plot(all_t, all_x, 'r-')
ax2.set_ylabel('X position (m)', color='r')
ax2.tick_params('y', colors='r')

plt.savefig('G_x_y_vs_time.png', dpi=graph_dpi)

# X versus Y

plt.figure()
fig, ax1 = plt.subplots()
fig.set_size_inches(11, 11)

ax1.set_xlabel('X position (m)')
ax1.set_ylabel('Y position (m)')
xymax = max(max(all_x), max(all_y)) + 200000  # find highest value in both sets
xymin = min(min(all_x), min(all_y)) - 200000  # find lowest value
if xymin < 0:                                 # if lowest is below 0 it might need a bigger frame
  xymax = max(xymax, 0-xymin)

ax1.set_ylim([0-xymax,xymax])
ax1.set_xlim([0-xymax,xymax])

def gimmeACircle(radius):
    circleX = []
    circleY = []
    for degRange in range(0, 360, 1):
        circleX.append(radius * math.sin(math.radians(degRange)))
        circleY.append(radius * math.cos(math.radians(degRange)))
    return circleX, circleY

earthSurfaceCircleX, earthSurfaceCircleY = gimmeACircle(planetRadius)
topAtmosLayerX, topAtmosLayerY = gimmeACircle(planetRadius + 86000)

ax1.scatter(all_x, all_y, s=3, c='red')
ax1.scatter(earthSurfaceCircleX, earthSurfaceCircleY, s=3, c='green')
ax1.scatter(topAtmosLayerX, topAtmosLayerY, s=1, c='blue')

if timeIndexAtBurnout is not None:
  burnCoordX = [all_x[timeIndexAtBurnout]]
  burnCoordY = [all_y[timeIndexAtBurnout]]
  ax1.scatter(burnCoordX, burnCoordY, s=50, c='red')
  ax1.scatter(burnCoordX, burnCoordY, s=20, c='white')

plt.savefig('G_x_y_scatter.png', dpi=graph_dpi)

# Accel, ax, and ay graph

plt.figure()
fig2, ax1 = plt.subplots()
fig2.set_size_inches(11, 5)

if timeIndexAtBurnout is not None:
  burnCoordX = [all_t[timeIndexAtBurnout]]
  burnCoordY = [all_accel[timeIndexAtBurnout]]
  ax1.scatter(burnCoordX, burnCoordY, s=50, c='blue')
  ax1.scatter(burnCoordX, burnCoordY, s=20, c='white')

ax1.plot(all_t, all_accel, 'b-')
ax1.set_xlabel('time (s)')
ax1.set_ylabel('Accel (m/s2)', color='b')
ax1.tick_params('y', colors='b')

ax2 = ax1.twinx()

if timeIndexAtBurnout is not None:
  burnCoordX = [all_t[timeIndexAtBurnout]]
  burnCoordY = [all_Ax[timeIndexAtBurnout]]
  ax2.scatter(burnCoordX, burnCoordY, s=50, c='red')
  ax2.scatter(burnCoordX, burnCoordY, s=20, c='white')

if timeIndexAtBurnout is not None:
  burnCoordX = [all_t[timeIndexAtBurnout]]
  burnCoordY = [all_Ay[timeIndexAtBurnout]]
  ax2.scatter(burnCoordX, burnCoordY, s=50, c='green')
  ax2.scatter(burnCoordX, burnCoordY, s=20, c='white')

ax2.plot(all_t, all_Ax, 'r-')
ax2.plot(all_t, all_Ay, 'g-')
ax2.set_ylabel('Accel along X axis', color='r')
ax2.set_ylabel('Accel along Y axis', color='g')
ax2.tick_params('y', colors='r')

plt.savefig('G_accel.png', dpi=graph_dpi)

# Air density and height and pressure

plt.figure()
fig2, ax1 = plt.subplots()
fig2.set_size_inches(11, 5)

ax1.plot(all_t, all_localAirDensity, 'b-')
ax1.set_xlabel('time (s)')
ax1.set_ylabel('Local air density', color='b')
ax1.tick_params('y', colors='b')

ax2 = ax1.twinx()
ax2.plot(all_t, all_localPressure, 'g-')
ax2.set_ylabel('Local pressure (Pascals)', color='g')
ax2.tick_params('y', colors='g')

ax2 = ax1.twinx()
ax2.plot(all_t, all_h, 'r-')
ax2.set_ylabel('Height (m)', color='r')
ax2.tick_params('y', colors='r')

plt.savefig('G_local_air_density-height.png', dpi=graph_dpi)

# VelocityX and VelocityY

plt.figure()
fig2, ax1 = plt.subplots()
fig2.set_size_inches(11, 5)

ax1.plot(all_t, all_VelocityX, 'b-')
ax1.set_xlabel('time (s)')
ax1.set_ylabel('VelocityX', color='b')
ax1.tick_params('y', colors='b')

if timeIndexAtBurnout is not None:
  burnCoordX = [all_t[timeIndexAtBurnout]]
  burnCoordY = [all_VelocityX[timeIndexAtBurnout]]
  ax1.scatter(burnCoordX, burnCoordY, s=50, c='blue')
  ax1.scatter(burnCoordX, burnCoordY, s=20, c='white')

ax2 = ax1.twinx()
ax2.plot(all_t, all_VelocityY, 'g-')
ax2.set_ylabel('VelocityY', color='g')
ax2.tick_params('y', colors='g')

if timeIndexAtBurnout is not None:
  burnCoordX = [all_t[timeIndexAtBurnout]]
  burnCoordY = [all_VelocityY[timeIndexAtBurnout]]
  ax2.scatter(burnCoordX, burnCoordY, s=50, c='green')
  ax2.scatter(burnCoordX, burnCoordY, s=20, c='white')

plt.savefig('G_velocityx_velocityy.png', dpi=graph_dpi)

# forceOfDrag and FlightPathDeg and HeadingDeg

plt.figure()
fig2, ax1 = plt.subplots()
fig2.set_size_inches(11, 5)

ax1.plot(all_t, all_forceOfDrag, 'b-')
ax1.set_xlabel('time (s)')
ax1.set_ylabel('forceOfDrag', color='b')
ax1.tick_params('y', colors='b')

ax2 = ax1.twinx()
ax2.plot(all_t, all_FlightPathDeg, 'g-')
ax2.set_ylabel('FlightPathDeg', color='g')
ax2.tick_params('y', colors='g')

ax2 = ax1.twinx()
ax2.plot(all_t, all_HeadingDeg, 'r-')
ax2.set_ylabel('HeadingDeg', color='r')
ax2.tick_params('y', colors='r')

plt.savefig('G_forceOfDrag_FlightPathDeg_Head.png', dpi=graph_dpi)
#plt.savefig('G_Kick-%s-KickHeight-%s-forceOfDrag_FlightPathDeg_Head.png' % (kickAngleDeg,kickHeight),dpi = graph_dpi)

# ThrustN

plt.figure()
fig2, ax1 = plt.subplots()
fig2.set_size_inches(11, 5)

ax1.plot(all_t, all_ThrustN, 'b-')
ax1.set_xlabel('time (s)')
ax1.set_ylabel('ThrustN', color='b')
ax1.tick_params('y', colors='b')

if timeIndexAtBurnout is not None:
  burnCoordX = [all_t[timeIndexAtBurnout]]
  burnCoordY = [all_ThrustN[timeIndexAtBurnout]]
  ax1.scatter(burnCoordX, burnCoordY, s=50, c='blue')
  ax1.scatter(burnCoordX, burnCoordY, s=20, c='white')

plt.savefig('G_ThrustN.png', dpi=graph_dpi)

# GravAng

plt.figure()
fig2, ax1 = plt.subplots()
fig2.set_size_inches(11, 5)

ax1.plot(all_t, all_GravAng, 'b-')
ax1.set_xlabel('time (s)')
ax1.set_ylabel('GravAng', color='b')
ax1.tick_params('a', colors='b')

ax2 = ax1.twinx()
ax2.plot(all_t, all_GravForce, 'g-')
ax2.set_ylabel('GravForce', color='g')
ax2.tick_params('y', colors='g')

plt.savefig('G_GravAng_GravForce.png', dpi=graph_dpi)

#Position
#plt.figure()
#ax = plt.subplot(111, polar=True)
#stepped_d = []
#stepped_m = []
#skip = 0
#index = 0
#while index < len(all_PolarCoordDeg):
#   d = all_PolarCoordDeg[index]
#    m = all_PolarCoordMag[index]
#    skip = skip + 1
#    if skip > 50:
#        stepped_d.append(d)
#        stepped_m.append(m)
#        skip = 0
#    index = index + 1
#ax.scatter(all_PolarCoordDeg, all_PolarCoordMag, s=1.1, c='red')
#ax.plot(q, all_PolarCoordMag, 'r-')
#ax.scatter(stepped_d, stepped_m, s=2, c='black')
#print (all_PolarCoordDeg)
#plt.savefig('G_radial_flight_path.png',dpi = 700)
#plt.savefig('G_Kick-%s-Height-%s-flight_path.png' % (kickAngleDeg,kickHeight),dpi = 1500)

plt.figure()
fig2, ax1 = plt.subplots()
fig2.set_size_inches(11, 5)

ax1.plot(all_t, all_dynamicQ, 'b-')
ax1.set_xlabel('time (s)')
ax1.set_ylabel('Dynamic Pressure', color='b')
ax1.tick_params('y', colors='b')

if timeIndexAtBurnout is not None:
  burnCoordX = [all_t[timeIndexAtBurnout]]
  burnCoordY = [all_dynamicQ[timeIndexAtBurnout]]
  ax1.scatter(burnCoordX, burnCoordY, s=50, c='blue')
  ax1.scatter(burnCoordX, burnCoordY, s=20, c='white')

plt.savefig('G_DynamicQ.png', dpi=graph_dpi)
