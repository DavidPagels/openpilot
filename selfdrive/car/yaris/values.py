# flake8: noqa

from selfdrive.car import dbc_dict
from cereal import car
from selfdrive.config import Conversions as CV

Ecu = car.CarParams.Ecu
MIN_ACC_SPEED = 19. * CV.MPH_TO_MS  # for non-stop-and-go cars (eg. needing gas interceptor)

class CarControllerParams:
  ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
  ACCEL_MAX = 1.5  # 1.5 m/s2
  ACCEL_MIN = -3.0  # 3   m/s2
  ACCEL_SCALE = max(ACCEL_MAX, -ACCEL_MIN)

  STEER_MAX = 1500
  STEER_DELTA_UP = 10       # 1.5s time to peak torque
  STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor

class CAR:
  YARIS = "TOYOTA YARIS 2008"

  # addr: (ecu, cars, bus, 1/freq*100, vl)
STATIC_MSGS = [
  (0x130, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x00\x00\x00\x00\x00\x00\x38'),
  (0x240, Ecu.fwdCamera, (CAR.YARIS), 0,   5, b'\x00\x10\x01\x00\x10\x01\x00'),
  (0x241, Ecu.fwdCamera, (CAR.YARIS), 0,   5, b'\x00\x10\x01\x00\x10\x01\x00'),
  (0x244, Ecu.fwdCamera, (CAR.YARIS), 0,   5, b'\x00\x10\x01\x00\x10\x01\x00'),
  (0x245, Ecu.fwdCamera, (CAR.YARIS), 0,   5, b'\x00\x10\x01\x00\x10\x01\x00'),
  (0x248, Ecu.fwdCamera, (CAR.YARIS), 0,   5, b'\x00\x00\x00\x00\x00\x00\x01'),
  (0x367, Ecu.fwdCamera, (CAR.YARIS), 0,  40, b'\x06\x00'),
  (0x414, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x00\x00\x00\x00\x00\x00\x17\x00'),
  (0x466, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x24\x20\xB1'),
  (0x489, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x00\x00\x00\x00\x00\x00\x00'),
  (0x48a, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x00\x00\x00\x00\x00\x00\x00'),
  (0x48b, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x66\x06\x08\x0a\x02\x00\x00\x00'),
  (0x4d3, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x1C\x00\x00\x01\x00\x00\x00\x00'),
  (0x4c6, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x07\x00\x03\x00\x00\x00\x00\x00'),
  (0x3b1, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x00\x00\x00\x01\x00\x08\x00\x00'),
  (0x1c4, Ecu.fwdCamera, (CAR.YARIS), 0, 2, b'\x05\xea\x1b\x08\x00\x00\xc0\x9f'),
  # (0x1d2, Ecu.fwdCamera, (CAR.YARIS), 0, 3, b'\xf8\x30\xf9\xe8\x00\x54\x80\xb8'),
  # (0x1d3, Ecu.fwdCamera, (CAR.YARIS), 0, 3, b'\x00\xa8\x41\x81\x16\x00\x00\x5c'),
  (0x2c1, Ecu.fwdCamera, (CAR.YARIS), 0, 3, b'\x08\x07\x07\x06\x70\xf8\x00\x4f'),
  (0x3d3, Ecu.fwdCamera, (CAR.YARIS), 0, 50, b'\x00\x00'),
  (0x399, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x00\x00\x00\x00\x00\x00\x00\x00'),
  (0x3bb, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x00\x00\x26\x00'),
  (0x3f9, Ecu.fwdCamera, (CAR.YARIS), 0, 20, b'\x76\x18\x26\x01\x00\x00\x00\xb9'),
  (0x3bc, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x00\x00\x00\x00\x00\x80\x00\x00'),
  (0x24, Ecu.fwdCamera, (CAR.YARIS), 0, 1, b'\x02\x00\x01\xfd\x42\x03\x80\xf1'),
  (0x3b1, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x00\x00\x00\x01\x00\x08\x00\x00'),
  (0x4ac, Ecu.fwdCamera, (CAR.YARIS), 0, 50, b'\x28\x00\x60\x01\x0a\x00\xa3\xa0'),

  (0x128, Ecu.fwdCamera, (CAR.YARIS), 0,   3, b'\xf4\x01\x90\x83\x00\x37'),
  (0x141, Ecu.fwdCamera, (CAR.YARIS), 0,   2, b'\x00\x00\x00\x46'),
  (0x160, Ecu.fwdCamera, (CAR.YARIS), 0,   7, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, Ecu.fwdCamera, (CAR.YARIS), 0,   7, b'\x00\x1e\x00\x00\x00\x80\x07'),
  (0x283, Ecu.fwdCamera, (CAR.YARIS), 0,   3, b'\x00\x00\x00\x00\x00\x00\x8c'),

  (0x344, Ecu.fwdCamera, (CAR.YARIS), 0,   5, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, Ecu.fwdCamera, (CAR.YARIS), 0,  20, b'\x00\x00\x00\x80\xfc\x00\x08'),
  (0x366, Ecu.fwdCamera, (CAR.YARIS), 0,  20, b'\x00\x72\x07\xff\x09\xfe\x00'),
  (0x4CB, Ecu.fwdCamera, (CAR.YARIS), 0, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
]

ECU_FINGERPRINT = {
  Ecu.fwdCamera: [0x423],   # steer torque cmd
  Ecu.dsu: [0x283],   # accel cmd
}

FINGERPRINTS = {
  CAR.YARIS: [{
    37: 8, 705: 8, 608: 8, 708: 8, 720: 8, 610: 5, 849: 4, 896: 8, 928: 8, 1088: 8, 1552: 8, 1568: 8, 916: 2, 945: 8, 1090: 8, 1217: 8, 948: 7, 1222: 8, 1059: 1, 947: 3, 1224: 8, 906: 5, 1244: 8, 1553: 8, 944: 5, 1600: 8, 1569: 8, 1570: 8, 1584: 8, 1245: 8, 1592: 8, 929: 8, 920: 2, 466: 8, 467: 8, 170: 8, 706: 8, 548: 8, 951: 8, 180: 1
  }]
}

IGNORED_FINGERPRINTS = []

FW_VERSIONS = {}

STEER_THRESHOLD = 100

DBC = {
  CAR.YARIS: dbc_dict('toyota_yaris_2008', 'toyota_adas')
}

TSS2_CAR = set([])
NO_DSU_CAR = set([CAR.YARIS])
NO_STOP_TIMER_CAR = set([CAR.YARIS])
