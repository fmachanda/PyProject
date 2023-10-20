class PID:
    def __init__(
            self, 
            kp: float=0.0, 
            ti: float=0.0, 
            td: float=0.0, 
            integral_limit: float = None, 
            minimum: float = None, 
            maximum: float = None) -> None:
        self.kp = kp
        self.ti = ti
        self.td = td

        self.integral_limit = integral_limit
        self.minimum = minimum
        self.maximum = maximum

        self._proportional = 0.0
        self._integral = 0.0
        self._derivative = 0.0

        self._error = 0.0
        self.output = 0.0

    def set(
            self, 
            kp: float = None, 
            ti: float = None, 
            td: float = None, 
            integral_limit: float = None, 
            minimum: float = None, 
            maximum: float = None) -> None:
        if kp is not None:
            self.kp = kp
        if ti is not None:
            self.ti = ti
        if td is not None:
            self.td = td

        if integral_limit is not None:
            self.integral_limit = integral_limit
        if minimum is not None:
            self.minimum = minimum
        if maximum is not None:
            self.maximum = maximum

    def reset(self) -> None:
        self._proportional = 0.0
        self._integral = 0.0
        self._derivative = 0.0

        self._error = 0.0
        self.output = 0.0

    def cycle(self, value: float, setpoint: float, time_step: float) -> float:
        ki = (self.kp / self.ti) if (self.ti != 0.0) else 0.0

        time_step /= 1e6
        time_step = max(1e-6, time_step)

        error = setpoint - value

        self._proportional = error

        self._integral += (ki * error * time_step)

        if self.integral_limit is not None:
            self._integral = min(self._integral,  abs(self.integral_limit))
            self._integral = max(self._integral, -abs(self.integral_limit))

        self._derivative = self.td * (error - self._error) / time_step

        output = (self._proportional + self._integral + self._derivative) * self.kp
        
        self._error = error

        if self.minimum is not None:
            output = max(output, self.minimum)

        if self.maximum is not None:
            output = min(output, self.maximum)

        self.output = output
        return self.output