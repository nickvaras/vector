from ..action import ReportAction


class ReportActionBTSignal(ReportAction):
    """Warns when a low report rate is discovered and may impact usability."""

    def __init__(self, *args, **kwargs):
        super(ReportActionBTSignal, self).__init__(*args, **kwargs)

        self.timer_check = self.create_timer(1.0, self.check_signal)
        self.timer_reset = self.create_timer(10, self.reset_warning)
        self.rps = 250

    def setup(self, device):
        self.reports = 0
        self.signal_warned = False

        if device.type == "bluetooth":
            self.enable()
        else:
            self.disable()

    def enable(self):
        self.timer_check.start()

    def disable(self):
        self.timer_check.stop()
        self.timer_reset.stop()

    def check_signal(self, report):
        # Less than 60 reports/s means we are probably dropping
        # reports between frames in a 60 FPS game.
        self.rps = int(self.reports / 1.0)
        
        if (self.rps < 60):
            if not self.signal_warned:
                self.logger.warning("Signal strength is low ({0} reports/s)", self.rps)
                self.signal_warned = True
                self.timer_reset.start()
       
        self.reports = 0

        return True

    def reset_warning(self, report):
        self.signal_warned = False

    def handle_report(self, report):
        self.reports += 1
