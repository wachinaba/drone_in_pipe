from rospy import Subscriber, Publisher
from sensor_msgs.msg import Range
from mavros_msgs.msg import RCIn, OverrideRCIn


class PID:
    def __init__(self, p, i, d) -> None:
        self.gain_p, self.gain_i, self.gain_d = p, i, d

    def calc()


class Converter:
    def __init__(self, pwm_min, pwm_max) -> None:
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max

    def convert(self, value) -> float:
        


class AutoflightNode:
    RC_CHANNEL_THRUST = 2
    PID_GAIN_P = 0.1
    PID_GAIN_I = 0.1
    PID_GAIN_D = 0.1

    def __init__(self) -> None:
        self.altitude = 0.0
        self.target_altitude = 0.0
        self.rcin = RCIn()

        self.pid_controller = PID(self.PID_GAIN_P, self.PID_GAIN_I, self.PID_GAIN_D)

        self.rangefinder_sub = Subscriber("/rangefinder_bottom", Range, self.rangefinder_cb)
        self.rcin_sub = Subscriber("/mavros/rc/in", RCIn, self.rcin_cb)
        self.override_pub = Publisher("/mavros/rc/override", OverrideRCIn, queue_size=5)

    def spin_once(self) -> None:


    def rangefinder_cb(self, msg: Range) -> None:
        self.altitude = msg.range

    def rcin_cb(self, msg: RCIn) -> None:
        self.rcin = msg

    
