from ..utils import IKFastInfo
from ..ikfast import *

#FRANKA_URDF = "models/franka_description/robots/panda_arm_hand.urdf"
SPOT_URDF = "models/spot_description/spot_arm.urdf"

#PANDA_INFO = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='panda_link0',
#                        ee_link='panda_link8', free_joints=['panda_joint7'])

SPOT_INFO = IKFastInfo(module_name='spot.ikfast_spot_arm', base_link='base',
                        ee_link='arm0.link_wr1', free_joints=['arm0.wr0'])
