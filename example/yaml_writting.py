import os
import sys

script_dir = os.path.dirname(__file__)
mymodule_dir = os.path.join(script_dir, "../utils")
sys.path.append(mymodule_dir)

from loader_tools import generateYAML



path_to_robot=os.getcwd() + "/robots/digit_like"
motor_name='mot'
closed_loop_frame_name="fermeture"
spherical_joint_name="to_rotule"

generateYAML(path_to_robot,motor_name,spherical_joint_name,closed_loop_frame_name)
