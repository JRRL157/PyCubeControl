import numpy as np

class Comparator:

    def __init__(self, input1_sgn: bool, input2_sgn: bool):
        self.input1_sgn = input1_sgn
        self.input2_sgn = input2_sgn
        self.output = 1e99

    def compare(self, input1, input2):
        if self.input1_sgn and self.input2_sgn:
            self.output = input1 + input2
        elif not self.input1_sgn and self.input2_sgn:
            self.output = input2 - input1
        elif self.input1_sgn and not self.input2_sgn:
            self.output = input1 - input2
        else:
            self.output = -input1 - input2
