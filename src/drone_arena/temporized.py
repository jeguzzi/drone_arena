import rospy


class Temporized(object):

    def __init__(self, period):
        # type: (float) -> None
        self.period = period
        self.last = None

    def __call__(self, f, *args, **kwargs):  # type: ignore
        def g(*args, **kwargs):  # type: ignore
            if self.last and (rospy.Time.now() - self.last).to_sec() < self.period:
                return
            self.last = rospy.Time.now()
            return f(*args, **kwargs)
        return g
