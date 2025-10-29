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
        
        height = 0.2 # 20cm heigh

        instructions[0] = [0, 0, -(height - ground_offset + self.handle_offset)]

        return np.array(instructions)
    
    def get_instructions_B(self, angle):
        """
        Returns instructions for labyrint type B.

        By default the final instruction is 2cm above base
        """
        a = np.deg2rad(angle)
        rotation = np.array([
            [np.cos(a), np.sin(a), 0],
            [-np.sin(a), np.cos(a), 0],
            [0, 0, 1]
        ])

        instructions = [
            [0.07, 0, 0],
            [0, 0, -0.06],
            [-0.07, 0, -0.07],
            [0, 0, -0.06]  
        ]

        for i, instruction in enumerate(instructions):
            instructions[i] = rotation @ np.array(instruction)

        return np.array(instructions)

    def get_instructions_B(self, angle):


        pass