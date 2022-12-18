class LPF:
    def __init__(self, blend) -> None:
        self.prev_output = 0.0
        self.blend = blend

    def update(self, input) -> float:
        output = input * self.blend + self.prev_output * (1 - self.blend)
        self.prev_output = output
        return output


class PIDController:
    def __init__(self, p, i, d, constant, filter, i_limit) -> None:
        self.p = p
        self.i = i
        self.d = d
        self.constant = constant
        self.filter = filter
        self.prev_input = 0.0
        self.prev_error = 0.0
        self.i_output = 0.0
        self.p_output = 0.0
        self.d_output = 0.0
        self.output = 0.0
        self.i_limit = i_limit

    def update(self, input, target) -> float:
        error = target - input
        self.i_output = min(self.i_limit, max(-self.i_limit, self.i_output + error * self.i))
        self.p_output = error * self.p
        self.d_output = self.filter.update(self.prev_input - input) * self.d

        self.output = self.p_output + self.i_output + self.d_output + self.constant
        self.prev_error = error
        self.prev_input = input

        return self.output
