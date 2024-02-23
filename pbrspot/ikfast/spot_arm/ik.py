from ..utils import IKFastInfo
from ..ikfast import *

SPOT_URDF = "models/spot_description/spot_arm.urdf"

#SPOT_INFO = IKFastInfo(module_name='spot.ikfast_spot_arm', base_link='body',
#                        ee_link='arm_link_wr1', free_joints=['arm_wr0'])
SPOT_INFO = IKFastInfo(module_name='spot.ikfast_spot_arm', base_link='body',
                        ee_link='arm_link_wr1', free_joints=[])

