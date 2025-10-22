import numpy as np


class TargetHandler():
    def __init__(self):
        self.handle_offset = 0.02 # 2cm above target

    def get_instructions_A(self, ground_offset=0.02):
        """
        Returns instructions for labyrint type A.

        By default the final instruction is 2cm above base
        """
        instructions = np.array([])
        
        height = 0.2

        instructions[0] = -(height - ground_offset + self.handle_offset)

        return instructions

