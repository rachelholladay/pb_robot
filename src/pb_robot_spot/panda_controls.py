import time
import numpy
import pybullet as p
import pb_robot_spot

class PandaControls(object):
    def __init__(self, arm):
        self.arm = arm
        print("Set up")

    def clampTorque(self, tau_d):
        tau_limit = [87, 87, 87, 87, 50, 50, 50] #robot.arm.torque_limits
        for i in range(len(tau_d)):
            tau_d[i] = pb_robot_spot.helper.clip(tau_d[i], -tau_limit[i], tau_limit[i])
        return tau_d

    def positionControl(self, q, threshold=0.1):
        n = len(q)
        while True:
            p.setJointMotorControlArray(self.arm.bodyID, self.arm.jointsID,
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=q,
                            targetVelocities=[0]*n,
                            forces=self.arm.torque_limits,
                            positionGains=[0.1]*n,
                            velocityGains=[1]*n)
            p.stepSimulation()
            time.sleep(0.01)

            if numpy.linalg.norm(numpy.subtract(self.arm.GetJointValues(), q)) < threshold:
                break

    def moveToTouch(self, q_desired):
        n = len(q_desired)
        p.stepSimulation()
        ft_past = p.getJointState(self.arm.bodyID, 8)[2]
        i = 0

        while True:
            p.setJointMotorControlArray(self.arm.bodyID, self.arm.jointsID,
                        controlMode=p.POSITION_CONTROL,
                        targetPositions=q_desired,
                        targetVelocities=[0]*n,
                        forces=self.arm.torque_limits,
                        positionGains=[0.1]*n,
                        velocityGains=[1]*n)
            p.stepSimulation()
            time.sleep(0.01)

            ft = self.arm.GetFTWristReading() #p.getJointState(robot.id, robot.ft_joint.jointID)[2]
            # So the in collision check seems incredibly accurate
            # The FT sensor check produces false positives (and stops trigging once in collision
            # So for now its check FT and then confirm with is collision. 
            if (numpy.linalg.norm(numpy.subtract(ft, ft_past)) > 100):
                if not self.arm.IsCollisionFree(self.arm.GetJointValues()):
                    break

            if numpy.linalg.norm(numpy.subtract(self.arm.GetJointValues(), q_desired)) < 0.01:
                if self.arm.IsCollisionFree(self.arm.GetJointValues()):
                    raise RuntimeError("MoveToTouch did not end in contact")

            ft_past = copy.deepcopy(ft)

    def forceControl(self, wrench_target, timeExerted):
        # wrench target needs to be a column, not row vector
        # Time exerted is in seconds
        wrench_desired = numpy.zeros((6, 1))
        gain = 0.1 #Was 0.01
        p.setGravity(0, 0, -9.81)
        # To go down, z should be negative! 

        fts = [0]*1000
        jfs = [0]*1000
        counts = int(timeExerted / 0.01) # 0.01 is rate

        for i in range(counts):
            p.setJointMotorControlArray(self.arm.bodyID, self.arm.jointsID, p.VELOCITY_CONTROL,
                                        targetVelocities=[0]*7, forces=[35]*7)

            jacobian = self.arm.GetJacobian(self.arm.GetJointValues())
            # Feedforward. Commented out PI control because gains were 0 
            tau_d = numpy.matmul(numpy.transpose(jacobian), wrench_desired)
            tau_cmd = self.clampTorque(tau_d)

            p.setJointMotorControlArray(self.arm.bodyID, self.arm.jointsID,
                        controlMode=p.TORQUE_CONTROL,
                        forces=tau_cmd)

            wrench_desired = gain * wrench_target + (1 - gain) * wrench_desired
            p.stepSimulation()
            time.sleep(0.01)

            fts[i] = self.arm.GetFTWristReading()[2]
            jfs[i] = numpy.matmul(self.arm.GetJacobian(self.arm.GetJointValues()), self.arm.GetJointTorques())[2]
            # Large difference between commanded torque and observed torque (with or without gravity)
            # Is this true on the real robot? (i dont know!)

    def jointImpedance(self, q_d_target, stiffness_params):
        # Seems to work but requires huuuge stiffness coefficients to get to pose (much higher than on real robot)
        p.setGravity(0, 0, -9.81)

        dq_d_target = numpy.zeros(len(q_d_target))
        stiffness_target = numpy.diag(stiffness_params)
        damping_target = numpy.diag(2.0*numpy.sqrt(stiffness_params))
        stiffness = numpy.eye(7)
        damping = numpy.eye(7)
        gain = 0.1

        q_d = self.arm.GetJointValues()
        dq_d = self.arm.GetJointVelocities()
        N = len(q_d)

        prev_q = self.arm.GetJointValues()
        diff_count = 0
        eps = 1e-4

        while True:
            diff = numpy.linalg.norm(numpy.subtract(self.arm.GetJointValues(), prev_q), ord=2)
            if diff < eps:
                diff_count += 1
            else:
                diff_count = 0
            if diff_count > 20:
                break

            p.setJointMotorControlArray(self.arm.bodyID, self.arm.jointsID, p.VELOCITY_CONTROL,
                                        targetVelocities=[0]*7, forces=[40]*7)

            tau_d = [0]*N
            prev_q = self.arm.GetJointValues()
            q = self.arm.GetJointValues()
            dq = self.arm.GetJointVelocities()
            coriolis = self.arm.GetCoriolosMatrix(q, dq)

            for i in range(N):
                tau_d[i] = coriolis[i] + (stiffness[i,i]*(q_d[i] - q[i])) + (damping[i,i]*(dq_d[i] - dq[i]))

            tau_cmd = self.clampTorque(tau_d)
            p.setJointMotorControlArray(self.arm.bodyID, self.arm.jointsID,
                                        controlMode=p.TORQUE_CONTROL,
                                        forces=tau_cmd)

            stiffness = gain * stiffness_target + (1 - gain) * stiffness
            damping = gain * damping_target + (1 - gain) * damping
            q_d = gain * q_d_target + (1 - gain) * q_d
            dq_d = gain * dq_d_target + (1 - gain) * dq_d

            p.stepSimulation()
            time.sleep(0.01)

    def cartImpedance(self, pose_d_target, stiffness_params):
        p.setGravity(0, 0, -9.81)

        position_d_target = pose_d_target[0:3, 3]
        ori_d_target = pb_robot_spot.geometry.quat_from_matrix(pose_d_target[0:3, 0:3])
        stiffness_target = numpy.diag(stiffness_params)
        damping_target = numpy.diag(2.0*numpy.sqrt(stiffness_params))

        stiffness = numpy.eye(6)
        damping = numpy.eye(6)
        position_d = self.arm.GetEETransform()[0:3, 3]
        ori_d = pb_robot_spot.geometry.quat_from_matrix(self.arm.GetEETransform()[0:3, 0:3])
        gain = 0.1

        prev_q = self.arm.GetJointValues()
        diff_count = 0
        eps = 1e-4

        while True:
            diff = numpy.linalg.norm(numpy.subtract(self.arm.GetJointValues(), prev_q), ord=2)
            if diff < eps:
                diff_count += 1
            else:
                diff_count = 0
            if diff_count > 20:
                break

            p.setJointMotorControlArray(self.arm.bodyID, self.arm.jointsID, p.VELOCITY_CONTROL,
                                        targetVelocities=[0]*7, forces=[30]*7)

            # Compute Pose Error
            error = numpy.zeros((6))
            current_pose = self.arm.GetEETransform()
            position_error = current_pose[0:3, 3] - position_d
            error[0:3] = position_error

            current_ori = pb_robot_spot.geometry.quat_from_matrix(current_pose[0:3, 0:3])
            # Compute different quaternion
            error_ori_quat = pb_robot_spot.transformations.quaternion_multiply(current_ori, pb_robot_spot.transformations.quaternion_inverse(ori_d))
            # Convert to axis angle
            (error_ori_angle, error_ori_axis) = pb_robot_spot.geometry.quatToAxisAngle(error_ori_quat)
            ori_error = numpy.multiply(error_ori_axis, error_ori_angle)
            error[3:6] = ori_error

            q = self.arm.GetJointValues()
            prev_q = self.arm.GetJointValues()
            dq = self.arm.GetJointVelocities()
            jacobian = self.arm.GetJacobian(q)
            tau_task = (numpy.transpose(jacobian)).dot(-stiffness.dot(error) - damping.dot(jacobian.dot(dq)))
            coriolis = self.arm.GetCoriolosMatrix(q, dq)
            tau_d = numpy.add(tau_task, coriolis)
            tau_cmd = self.clampTorque(tau_d)

            p.setJointMotorControlArray(self.arm.bodyID, self.arm.jointsID,
                                        controlMode=p.TORQUE_CONTROL,
                                        forces=tau_cmd)

            # Update parameters
            stiffness = gain * stiffness_target + (1 - gain) * stiffness
            damping = gain * damping_target + (1 - gain) * damping
            position_d = gain * position_d_target + (1 - gain) * position_d

            (ori_d_angle, ori_d_axis) = pb_robot_spot.geometry.quatToAxisAngle(ori_d)
            (ori_d_target_angle, ori_d_target_axis) = pb_robot_spot.geometry.quatToAxisAngle(ori_d_target)
            ori_d_axis = gain * ori_d_target_axis + (1 - gain) * ori_d_axis
            ori_d_angle = gain * ori_d_target_angle + (1 - gain) * ori_d_angle
            ori_d = pb_robot_spot.geometry.quat_from_axis_angle(ori_d_axis, ori_d_angle)

            p.stepSimulation()
            time.sleep(0.01)

    def positionControlPath(self, path):
        for i in range(len(path)):
            self.positionControl(path[i])

    def jointImpedancePath(self, path, stiffness_params):
        for i in range(len(path)):
            self.jointImpedance(path[i], stiffness_params)

    def cartImpedancePath(self,path, stiffness_params):
        for i in range(len(path)):
            self.cartImpedance(path[i], stiffness_params)

