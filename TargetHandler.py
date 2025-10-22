import numpy as np


class TargetHandler():
    def __init__(self):
        self.handle_offset = 0.02 # 2cm above target

    def get_instructions_A(self, ground_offset=0.02):
        """
        Returns instructions for labyrint type A.

        By default the final instruction is 2cm above base
        """
        instructions = []
        
        height = 0.2 # 20cm height

        instructions[0] = [0, 0, -(height - ground_offset + self.handle_offset)]

        return np.array(instructions)
    
    def get_instructions_B(self, ground_offset=0.02):
        """
        Returns instructions for labyrint type A.

        By default the final instruction is 2cm above base
        """
        instructions = []

        height = 0.2

        instructions[0] = [0.07, 0, 0]
        instructions[1] = [0, 0, -(self.handle_offset + 0.05)]
        instructions[2] = [-0.07, 0, -0.07]
        instructions[3] = [0, 0, -0.08 + ground_offset]

        return np.array(instructions)

