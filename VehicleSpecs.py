import math
from mpmath import *
import numpy as np


class SLS():

    def getT2(self, p):
        return 9116000 - (16.5408339*p)


    def getLaunchMass(self, StageNo):
        return self.M1 + self.M2 + self.M3 + self.PayloadM + self.FairingMass;  # Current mass in kg


    def MassAndStageChanger(self, t, TimeStep, StageNo, localAirDensity, localPressure, V, M, MachNo):
      # Mass and Stage changer
      vT2 = self.getT2(localPressure)          # 7440000 N SL 9116000 N vac thrust in kn of stage 2
      if (t < self.Bt1):
        StageNo = 1
        ThrustN = self.T1 + vT2
        forceOfDrag = (2 * (.5 * localAirDensity * math.pow(V, 2) * self.CD1 * self.RefA1)) + (.5*localAirDensity * math.pow(V,2) * self.CD2 * self.RefA2)
        M = (M-((self.M1-self.Mo1) / self.Bt1)*TimeStep)
        M = (M-((self.M2-self.Mo2) / self.Bt2)*TimeStep)
      elif (t < self.Bt2):
        M = (M-((self.M2-self.Mo2) / self.Bt2)*TimeStep)
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.CD2 * self.RefA2)
        ThrustN = vT2
        if (StageNo == 1):
          M = M - self.Mo1
          StageNo = 2
      elif (t < (self.Bt3 + self.Bt2)):
        ThrustN = self.T3;
        M = (M-((self.M3-self.Mo3) / self.Bt3)*TimeStep)
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.CD3 * self.RefA3)
        if (StageNo == 2):
          StageNo = 3
          M = M - (self.Mo2 + self.FairingMass)
      else: # if (t > self.Bt3 + self.Bt2)
        ThrustN = 0
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.CD3 * self.RefA3)
        if self.timeAtBurnout is None:
            self.timeAtBurnout = t
            print ('Burnout at %s seconds' % t)
        if (StageNo == 3):
          StageNo = 4
          M = M - self.Mo3
      return M, ThrustN, forceOfDrag, StageNo


    def PitchControl(self, t, TimeStep, h, FlightPathDeg, HeadingDeg, PolarCoordDeg, dynamicQ):
        # Pitch Control Commands
        #kickHeight = 0
        #kickTime = 60
        #pitchEaseWindow = 50
        #kickAngleDeg = 36.4
        #pitch = 0

        #if (t >= kickTime) and (t <= kickTime + pitchEaseWindow):
        #    pitch = (kickAngleDeg / pitchEaseWindow) * (t - kickTime)
        #else:
        #    if FlightPathDeg < kickAngleDeg:
        #        pitch = kickAngleDeg
        #    else:
        #        pitch = FlightPathDeg

        if (dynamicQ > self.max_seen_q):
            self.max_seen_q = dynamicQ
        elif self.kickHeight is None:
            print ("Max Seen Q was %s , chosen kick height was %s" % (self.max_seen_q, h))
            self.kickHeight = h
        kickHeight = self.kickHeight
        kickAngleDeg = 20
        kickTransitionWindow = 20
        kickWindow = 100
        pitch = 0

        if kickHeight is not None:
            if h < kickHeight:
                pitch = 0
            elif h < kickHeight + kickTransitionWindow:
                pitch = (h - kickHeight) * (kickAngleDeg / kickTransitionWindow)
            else:
                if FlightPathDeg < kickAngleDeg:
                    pitch = kickAngleDeg
                else:
                    pitch = FlightPathDeg
        # if (h > 100000):
        #    if pitch > 90 + PolarCoordDeg:
        #        pitch = 90 + PolarCoordDeg
        # else:
        #    if pitch > 85 + PolarCoordDeg:
        #        pitch = 85 + PolarCoordDeg

        while pitch > 360:
            pitch = pitch - 360
        while pitch < 0:
            pitch = pitch + 360
        HeadingDeg = pitch
        return HeadingDeg, kickAngleDeg, kickHeight

class SLSBlock1(SLS):

    def __init__(self):
        # SLS Block 1 (ish)

        self.max_seen_q = -1
        self.kickHeight = None
        self.timeAtBurnout = None

        self.Mo1 = 200780  # dry mass in kg of stage 1
        self.Mo2 = 90275  # dry mass in kg of stage 2
        self.Mo3 = 4354  # dry mass in kg of Stage 3

        # Mass of each stage in kg
        self.M1 = 1463770  # wet mass in kg of stage 1
        self.M2 = 1091452  # wet mass in kg of stage 2
        self.M3 = 31207  # wet mass in kg of stage 3

        # thrust of each stage in  KN
        self.T1 = 32000000  # thrust in N of stage 1
        # T2 = 9116000 #- (16.5408339*P) # 7440000 N SL 9116000 N vac thrust in kn of stage 2
        self.T3 = 110100  # thrust in N of stage 3

        self.FairingMass = 4000  # fairing mass

        # Burn time of each stage in seconds
        self.Bt1 = 126
        self.Bt2 = 476
        self.Bt3 = 1125

        # Drag coeficiant of each stage
        self.CD1 = 0.394356099087654  # (for each booster)
        self.CD2 = None
        self.CD3 = None

        # Reference area of each stage
        self.RefA1 = 43.2411
        self.RefA2 = None
        self.RefA3 = None



class SLSBlock1B(SLS):

    def __init__(self):
        # SLS Block 1B (ish)

        self.max_seen_q = -1
        self.kickHeight = None
        self.timeAtBurnout = None

        self.Mo1 = 200780 # dry mass in kg of stage 1
        self.Mo2 = 85275  # dry mass in kg of stage 2
        self.Mo3 = 13092  # dry mass in kg of Stage 3 (interpolated)

        self.M1 = 1463770 # wet mass in kg of stage 1
        self.M2 = 1091452 # wet mass in kg of stage 2
        self.M3 = 142000  # wet mass in kg of stage 3 (interpolated)

        self.T1 = 32000000# thrust in N of stage 1
        #T2 = 9116000 #- (16.5408339*P) # 7440000 N SL 9116000 N vac thrust in kn of stage 2
        self.T3 = 440000  # thrust in N of stage 3

        self.FairingMass = 8000  # fairing mass

        #Burn time of each stage in seconds
        self.Bt1 = 126
        self.Bt2 = 476
        self.Bt3 = 5352

        # Drag coeficiant of each stage
        self.CD1 = 0.394356099087654 # (for each booster)
        self.CD2 = 0.46
        self.CD3 = 0.8

        # Reference area of each stage
        self.RefA1 = 43.2411
        self.RefA2 = 221.6705904
        self.RefA3 = 221.6705904

        self.PayloadM = 28600 #  ERV Earth Return Vehcicle mass at laucnh
        #self.PayloadM = 25200 #  Hab Habitiation/Lander Module mass at laucnh


class SLSBlock2(SLS):

    def __init__(self):
        # SLS Block 2 (ish)

        self.max_seen_q = -1
        self.kickHeight = None
        self.timeAtBurnout = None

        self.Mo1 = 168000 # dry mass in kg of stage 1
        self.Mo2 = 112000 # dry mass in kg of stage 2
        self.Mo3 = 13092  # dry mass in kg of Stage 3 (interpolated)

        self.M1 = 1586000 # wet mass in kg of stage 1
        self.M2 = 1091452 # wet mass in kg of stage 2
        self.M3 = 142000  # wet mass in kg of stage 3 (interpolated)

        self.T1 = 40000000# thrust in N of stage 1
        self.T3 = 440000  # thrust in N of stage 3

        self.FairingMass = 8000 #  fairing mass

        #Burn time of each stage in seconds
        self.Bt1 = 110
        self.Bt2 = 476
        self.Bt3 = 5352

        # Drag coeficiant of each stage
        self.CD1 = 0.394356099087654 # (for each booster)
        self.CD2 = 0.46
        self.CD3 = 0.8

        # Reference area of each stage
        self.RefA1 = 43.2411
        self.RefA2 = 221.6705904
        self.RefA3 = 221.6705904

        self.PayloadM = 28600 #  ERV Earth Return Vehcicle mass at laucnh
        #self.PayloadM = 25200 #  Hab Habitiation/Lander Module mass at laucnh


class SaturnV():

    def __init__(self):
        # Drag coefficients for every 0.5 Mach (starting at 0)
        self.dragCoefTable = [0.3, 0.26, 0.4, 0.55, 0.47, 0.36, 0.277, 0.23, 0.21, 0.205,
                         0.2, 0.205, 0.21, 0.22, 0.23, 0.24, 0.25, 0.257, 0.26, 0.26, 0.26]

        # Saturn V with  (ish)

        self.payload_LESmass = 4173 #Launch escape system mass in kg on front of payload

        self.max_seen_q = -1
        self.kickHeight = None
        self.timeAtBurnout = None

        self.pl_commMod = 5806   # Command module mass
        self.pl_serv = 24523  # Service module mass
        self.pl_lmAdapter = 1800   # LM adapter mass
        self.pl_LMtotalMass = 15200  # LM total mass
        self.payload_Apollo = self.pl_commMod + self.pl_serv + self.pl_lmAdapter + self.pl_LMtotalMass + self.payload_LESmass
        self.stage2_AftInterstageMass = 5195
        self.stage3_AftInterstageMass = 3650

        self.Mo1 = 130570  # dry mass in kg of stage 1
        self.Mo2 = 85275   # dry mass in kg of stage 2
        self.Mo3 = 13092   # dry mass in kg of Stage 3
        self.M1 = 2149500 + self.Mo1        # wet mass in kg of stage 1
        self.M2 = 451650 + self.Mo2 + 5195 # wet mass in kg of stage 2
        self.M3 = 106940 + self.Mo3 + 3650  # wet mass in kg of stage 3

        self.stageOneStartThrust = 34354772
        self.stageOneMaxThrust = 40064144
        self.stageOneOriginalStartLocalPressure = 165.075263457
        self.stageOneOriginalMaxLocalPressure = 100610

        #Math for determining thrust of first stage at a given pressure/altitude-+
        #ThrustN = stageOneMaxThrust - (((stageOneMaxThrust - stageOneStartThrust) /
        #                                (stageOneOriginalMaxLocalPressure - stageOneOriginalStartLocalPressure))
        #                               * localPressure)

        self.T2 = 5004000   # thrust in N of stage 2
        self.T3 = 1001000   # thrust in N of stage 3

        #Burn time and other event times of each stage in seconds
        self.F1RocketBurnRate = 2735.70738 #Burn rate of F-1 rocket engine in kg/s, inferred from burn times and fuel tank capacity
        self.J2RocketBurnRate = 306.59834 #Burn rate of J-2 rocket engine in kg/s, inferred from burn times and fuel tank capacity
        self.stage1_CenterEngCutoff = 135.2  # Center cuts off early at 135.5 reducing thrust and fuel consumption by 1/5
        self.stage1_OutboardEngCutoff = 162.63 # Outboard engine cutoff
        self.stage2_Ignition = 166
        self.stage2_AftInterstageJettisoned = 192.3
        self.stage2_LauchEscapeTowerJettison = 197.9
        self.stage2_CenterEngCutoff = 460.62
        self.stage2_OutboardEngCutoff = 548.22
        self.stage3_1stBurnStart = 560.2
        self.stage3_1stBurnCutoff = 699.33
        #self.stage3_2ndBurnStart = 9856.2
        #self.stage3_2ndBurnCutoff = 10203.03
        self.stage3_2ndBurnStart = 699.33       # Customized for out flightplan
        self.stage3_2ndBurnCutoff = 346.83+699.33

        # Drag coeficiant of each stage
        self.CD1 = 0.394356099087654
        self.CD2 = 0.46
        self.CD3 = 0.8

        # Reference area of each stage
        self.RefA1 = 112.97009664
        self.RefA2 = 79.48277735
        self.RefA3 = 34.2119151



    def getLaunchMass(self, StageNo):
        return self.M1 + self.M2 + self.M3 + self.payload_Apollo;  # Starting mass in kg


    def getT2(self, p):
        return 5004000


    def MassAndStageChanger(self, t, TimeStep, StageNo, localAirDensity, localPressure, V, M, MachNo):

      # Mass and Stage changer
      vT2 = self.getT2(localPressure)          # 7440000 N SL 9116000 N vac thrust in kn of stage 2
      if (t < self.stage1_CenterEngCutoff):
        StageNo = 1
        ThrustN = self.stageOneMaxThrust - (((self.stageOneMaxThrust - self.stageOneStartThrust) /
                                        (self.stageOneOriginalMaxLocalPressure - self.stageOneOriginalStartLocalPressure))
                                       * localPressure)
        M = self.M1 + self.M2 + self.M3 + self.payload_Apollo #- (self.F1RocketBurnRate * 5 * t)
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.dragCoefTable[int(min(MachNo, 10) * 2)] * self.RefA1)
      elif (t < self.stage1_OutboardEngCutoff):
        StageNo = 1
        ThrustN = (self.stageOneMaxThrust - (((self.stageOneMaxThrust - self.stageOneStartThrust) /
                                        (self.stageOneOriginalMaxLocalPressure - self.stageOneOriginalStartLocalPressure))
                                       * localPressure)) * 4/5
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.dragCoefTable[int(min(MachNo, 10) * 2)] * self.RefA1)
        M = self.M1 + self.M2 + self.M3 + self.payload_Apollo -\
            ((self.F1RocketBurnRate * 4 * (t - self.stage1_CenterEngCutoff) +
                (self.F1RocketBurnRate * 5 * self.stage1_CenterEngCutoff)))
      elif (t < self.stage2_Ignition):
        M = self.M2 + self.M3 + self.payload_Apollo
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.dragCoefTable[int(min(MachNo, 10) * 2)] * self.RefA1)
        ThrustN = 0
        StageNo = 2
      elif (t < (self.stage2_AftInterstageJettisoned)):
        M = self.M2 + self.M3 + self.payload_Apollo - (self.J2RocketBurnRate * 5 * (t - self.stage2_Ignition))
        ThrustN = self.T2
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.dragCoefTable[int(min(MachNo, 10) * 2)] * self.RefA2)
        StageNo = 2
      elif  (t < self.stage2_CenterEngCutoff):
        ThrustN = self.T2
        M = (self.M2 + self.M3 + self.payload_LESmass - self.stage2_AftInterstageMass) - (self.J2RocketBurnRate * 5 * (t - self.stage2_Ignition))
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.dragCoefTable[int(min(MachNo, 10) * 2)] * self.RefA2)
      elif  (t < self.stage2_OutboardEngCutoff):
        ThrustN = self.T2 * 4 / 5
        M = (M-((self.M2-self.Mo2) / self.stage2_OutboardEngCutoff)*TimeStep)  #need calc
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.dragCoefTable[int(min(MachNo, 10) * 2)] * self.RefA2)
        StageNo = 2
      elif  (t < self.stage3_1stBurnStart):
        M = self.M3 + self.payload_Apollo - (self.stage3_AftInterstageMass + self.payload_LESmass)
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.dragCoefTable[int(min(MachNo, 10) * 2)] * self.RefA3)
        ThrustN = self.T2
        StageNo = 3
      elif (t < self.stage3_1stBurnCutoff):
        M = self.M3 + self.payload_Apollo - (self.stage3_AftInterstageMass + self.payload_LESmass) - (self.J2RocketBurnRate * (t - self.stage3_1stBurnStart))
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.dragCoefTable[int(min(MachNo, 10) * 2)] * self.RefA3)
        ThrustN = self.T3
        StageNo = 3
      elif (t < self.stage3_2ndBurnStart):
        M = M = self.M3 + self.payload_Apollo - (self.stage3_AftInterstageMass + self.payload_LESmass) - (self.J2RocketBurnRate * (self.stage3_1stBurnCutoff - self.stage3_1stBurnStart))
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.CD2 * self.RefA2)
        ThrustN = 0
        StageNo = 3
      elif (t < self.stage3_2ndBurnCutoff):
        M = self.M3 + self.payload_Apollo - (self.stage3_AftInterstageMass + self.payload_LESmass) - ( (self.J2RocketBurnRate * (self.stage3_1stBurnCutoff - self.stage3_1stBurnStart)) + (t - self.stage3_2ndBurnStart))
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.dragCoefTable[int(min(MachNo, 10) * 2)] * self.RefA3)
        ThrustN = self.T3
        StageNo = 3
      else:
        if self.timeAtBurnout is None:
            self.timeAtBurnout = t
            print ('Burnout at %s seconds' % t)
        M = self.M3 + self.payload_Apollo - (self.stage3_AftInterstageMass + self.payload_LESmass) - ((self.J2RocketBurnRate * (self.stage3_1stBurnCutoff - self.stage3_1stBurnStart)) + (self.stage3_2ndBurnCutoff - self.stage3_2ndBurnStart))
        ThrustN = 0
        StageNo = 3
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * self.dragCoefTable[int(min(MachNo, 10) * 2)] * self.RefA3)
      return M, ThrustN, forceOfDrag, StageNo


    def PitchControl(self, t, TimeStep, h, FlightPathDeg, HeadingDeg, PolarCoordDeg, dynamicQ):
        # Pitch Control Commands
        #if (t < 0.3):
        #    ap = 0
        #elif (t < 30):
        #    ap = 0
        #elif (t < 80):
        #    ap = 0.728 * (t - 30)
        #elif (t < 135):
        #    ap = 36.40 + 0.469364 * (t - 80)
        #elif (t < 165):
        #    ap = 62.23 + 0.297 * (t - 135)
        #elif (t < 185):
        #    ap = 71.14 - 0.5285000 * (t - 165)
        #elif (t < 320):
        #    ap = 60.57 + 0.030963 * (t - 185)
        #elif (t < 460):
        #    ap = 64.75 + 0.09 * (t - 320)
        #elif (t < 480):
        #    ap = 77.35 - 0.138 * (t - 460)
        #elif (t < 550):
        #    ap = 74.59 + 0.0971429 * (t - 480)
        #elif (t < 570):
        #    ap = 81.39 - 0.207 * (t - 550)
        #elif (t < 640):
        #    ap = 77.25 + 0.1117143 * (t - 570)
        #elif (t < 705):
        #    ap = 85.07 + 0.0486154 * (t - 640)
        #else:
        #    ap = 88.23
        #return (ap - PolarCoordDeg)*0.55
        #k = .6
        #targetPerigee = 120000
        #pitch = 90 * math.pow(1 - (( h - 0) / (targetPerigee - 0)), (-k*h/targetPerigee)) - (90 + PolarCoordDeg)
        if (dynamicQ > self.max_seen_q):
            self.max_seen_q = dynamicQ
        elif self.kickHeight is None:
            print ("Max Seen Q was %s , chosen kick height was %s" % (self.max_seen_q, h))
            self.kickHeight = h
        kickHeight = self.kickHeight
        kickAngleDeg = 1.4
        kickTransitionWindow = 200
        kickWindow = 100
        pitch = 0

        if kickHeight is not None:
            if h < kickHeight:
                pitch = 0
            elif h < kickHeight + kickTransitionWindow:
                pitch = (h - kickHeight) * (kickAngleDeg / kickTransitionWindow)
            else:
                if FlightPathDeg < kickAngleDeg:
                    pitch = kickAngleDeg
                else:
                    pitch = FlightPathDeg
        #if (h > 100000):
        #    if pitch > 90 + PolarCoordDeg:
        #        pitch = 90 + PolarCoordDeg
        #else:
        #    if pitch > 85 + PolarCoordDeg:
        #        pitch = 85 + PolarCoordDeg

        while pitch > 360:
            pitch = pitch - 360
        while pitch < 0:
            pitch = pitch + 360
        HeadingDeg = pitch
        return HeadingDeg, kickAngleDeg, kickHeight