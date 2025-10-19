class PIController:
    def __init__(self, proportional_constant=0, integral_constant=0):
        self.proportional_constant = proportional_constant
        self.integral_constant = integral_constant
        self.integral_sum = 0
    
    def get_value(self, error):
        self.integral_sum += error
        return self.proportional_constant * error + self.integral_constant * self.integral_sum
    
    def reset(self):
        self.integral_sum = 0
