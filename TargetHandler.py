import numpy as np


class TargetHandler():
    def get_instructions_A(self, angle):
        instructions = [
            [0, 0, -0.18]
        ]

        return np.array(instructions)
    
    def get_instructions_B(self, angle):
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

    def get_instructions_C(self, angle):
        a = np.deg2rad(angle)
        rotation = np.array([
            [np.cos(a), np.sin(a), 0],
            [-np.sin(a), np.cos(a), 0],
            [0, 0, 1]
        ])

        instructions = [
            [-0.05, -0.05, 0],
            [0, 0, -0.06],
            [0, 0.05, -0.05],
            [0.05, 0, -0.05],
            [0, 0, -0.03]  
        ]

        for i, instruction in enumerate(instructions):
            instructions[i] = rotation @ np.array(instruction)

        return np.array(instructions)

    def get_instructions_D(self, angle):
        a = np.deg2rad(angle)
        rotation = np.array([
            [np.cos(a), np.sin(a), 0],
            [-np.sin(a), np.cos(a), 0],
            [0, 0, 1]
        ])

        instructions = [
            [0.115,0,0],
            [-0.05,0,0],
            [-0.03536,0,-0.01464],
            [-0.01464, 0,-0.03536],
            [0,0,-0.06],
            [0.015,0,-0.015],
            [0,0,-0.01]
        ]

        for i, instruction in enumerate(instructions):
            instructions[i] = rotation @ np.array(instruction)

        return np.array(instructions)