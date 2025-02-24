class RobotState:
    def _new_(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super(RobotState, cls)._new_(cls)
        return cls.instance
    def initialize(self, loaded = True):
        self.loaded = loaded
    def coralLoaded(self):
        return self.loaded