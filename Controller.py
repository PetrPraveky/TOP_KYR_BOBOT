from MoveHandler import *
from ImageHandler import *
from datetime import datetime


class Controller:
    def __init__(self, robot):
        self.robot = robot
        self.move_handler = MoveHandler(robot)
        self.image_handler = ImageHandler(robot)
        #self.calibration_relative_vectors = [(0.05, 0, 0), (0, 0.1, 0),(-0.2, 0, 0),(0, -0.2, 0), (0.2, 0, 0), (-0.1, 0.1, 0)]
        self.calibration_relative_vectors = [(0.05, 0, 0), (0, 0.05, 0), (0, 0.05, 0), (-0.05, 0, 0), (0, -0.05, 0),
                                             (0, -0.05, 0), (0, -0.05, 0), (0, -0.05, 0), (0.05, 0, 0), (0, 0.05, 0), (0, 0.05, 0), (-0.05, 0, 0)]
        self.calibration_absolute_vectors = []
        self.task2_instructions = [
            [0.07, 0, 0],
            [0, 0, -0.06],
            [-0.07, 0, -0.07],
            [0, 0, -0.06]
        ]

    
    def handle_input(self, raw_input):
        match raw_input[0]:
            case "p": # Robot should print something
                match raw_input[1:]:
                    case "fk":
                        print(self.robot.fk(self.robot.get_q()))
                    case "q":
                        print(np.rad2deg(self.robot.get_q()))
                    case "sq":
                        print(sum(np.rad2deg(self.robot.get_q()[:-1])))
                    case _:
                        print("Print command not found!")

            case "i": # Robot should do something with images
                match raw_input[1:]:
                    case "take":
                        self.image_handler.take_img()
                    case "show":
                        self.image_handler.show_img()
                    case "take_show":
                        self.image_handler.take_img()
                        self.image_handler.show_img()
                    case "save":
                        name = datetime.today().strftime('targets/%Y-%m-%d %H-%M-%S.png')
                        self.image_handler.save_img(filename=name)
                    case "take_save":
                        name = datetime.today().strftime('targets/%Y-%m-%d %H-%M-%S.png')
                        self.image_handler.take_img()
                        self.image_handler.save_img(filename=name)
                    case "calibrate":
                        self.calibrate()
                    case "target":
                        self.image_handler.find_target()
                    case _:
                        print("Img command not found!")

            case "m": # Robot should do some movement
                safety_command = None

                # while(safety_command != "y" and safety_command != "n"):
                #     safety_command = input("Are you sure you want to move the robot? press y/n:")
                # if safety_command != "y":
                #     return 0

                match raw_input[1:]:
                    case "task":
                        task_number = int(input("Enter task number"))
                        # move home
                        self.move_handler.move_home()
                        self.robot.wait_for_motion_stop()

                        # find target
                        self.image_handler.take_img()
                        self.image_handler.find_target()

                        # move to start
                        self.move_handler.move_to_q_position()
                        self.robot.wait_for_motion_stop()

                        # move to target
                        formated_pixels = np.array([[self.image_handler.target_pixels]], dtype=np.float64)
                        target_position = self.image_handler.pixels_to_position(formated_pixels)[0][0]
                        print("Target position:", target_position)
                        # up_first = self.robot.fk(self.robot.get_q())[:3,3]
                        # up_first[2] = 0.20
                        # self.move_handler.move_ik(up_first)
                        self.move_handler.move_ik(np.append(target_position,0.25))
                        self.robot.wait_for_motion_stop()

                        match task_number:
                            case 1:
                                self.move_handler.move_to_relative_position((0,0,-0.18))
                            case 2:
                                input("Is everything alright?")
                                self.move_handler.move_relative_sequantial(self.task2_instructions)
                    case "start":
                        self.move_handler.move_to_q_position()
                    case "home":
                        self.move_handler.move_home()
                    case "target":
                        formated_pixels = np.array([[self.image_handler.target_pixels]], dtype=np.float64)
                        target_position = self.image_handler.pixels_to_position(formated_pixels)[0][0]
                        print("Target position:", target_position)
                        up_first = self.robot.fk(self.robot.get_q())[:3,3]
                        up_first[2] = 0.25
                        self.move_handler.move_ik(up_first)
                        self.move_handler.move_ik(np.append(target_position,0.25))
                    case "w":
                        move_meters = float(input("How many centimeters do you want to move?"))/100
                        self.move_handler.move_to_relative_position((move_meters,0,0))
                    case "s":
                        move_meters = float(input("How many centimeters do you want to move?"))/100
                        self.move_handler.move_to_relative_position((-move_meters,0,0))
                    case "a":
                        move_meters = float(input("How many centimeters do you want to move?"))/100
                        self.move_handler.move_to_relative_position((0,move_meters,0))
                    case "d":
                        move_meters = float(input("How many centimeters do you want to move?"))/100
                        self.move_handler.move_to_relative_position((0,-move_meters,0))
                    case "q":
                        move_meters = float(input("How many centimeters do you want to move?"))/100
                        self.move_handler.move_to_relative_position((0,0,move_meters))
                    case "e":
                        move_meters = float(input("How many centimeters do you want to move?"))/100
                        self.move_handler.move_to_relative_position((0,0,-move_meters))
                    
                    case _:
                        print("Move command not found!")
            case _:
                print("Command not found!")

    def calibrate(self):
        self.move_handler.move_to_q_position(self.move_handler.CALIBRATION_POSITION) # Moves robot to starting position
        self.robot.wait_for_motion_stop()

        self.take_imgs_with_robot_movements(self.calibration_relative_vectors)

        self.image_handler.find_homography(self.calibration_absolute_vectors)

        self.image_handler.save_h()
    
    def take_imgs_with_robot_movements(self, move_vectors=None):
        if move_vectors == None:
            move_vectors = self.calibration_relative_vectors

        self.image_handler.imgs = []
        self.calibration_absolute_vectors = []

        for move_vector in move_vectors:
            self.calibration_absolute_vectors.append(self.robot.fk(self.robot.get_q())[:3,3])
            img = self.image_handler.take_img()
            self.image_handler.imgs.append(img)
            self.image_handler.show_img()
        
            self.move_handler.move_to_relative_position(move_vector)
            self.robot.wait_for_motion_stop()
            
        # Take last image
        self.calibration_absolute_vectors.append(self.robot.fk(self.robot.get_q())[:3,3])
        img = self.image_handler.take_img()
        self.image_handler.imgs.append(img)
        self.image_handler.show_img()

        return 0
    