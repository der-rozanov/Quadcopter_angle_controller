import numpy as np
import pylab

dw = 40

# -------------------------------------
g = 9.81
Timelapse = 4
dt = 0.01

wx_start = 0
wy_start = 0
wz_start = 0

ax_start = 0
ay_start = 0
az_start = 0

P_des = 500


# -------------------------------------

class QuadCopter:
    count = 0

    def __init__(self, angle, angular_velocity, mass=1, leng=0.15, inert=0.000001, z_inert_ratio=0.0000000001,
                 throttle_ratio=0.001, max_engine_rpm=20000, max_angular_velocity=10):
        self.coordinate_x = angle[0]
        self.coordinate_y = angle[1]
        self.coordinate_z = angle[2]

        self.velocity_x = angular_velocity[0]
        self.velocity_y = angular_velocity[1]
        self.velocity_z = angular_velocity[2]

        self.mass = mass
        self.leng = leng
        self.inert = inert
        self.z_inert_ratio = z_inert_ratio
        self.throttle_ratio = throttle_ratio
        self.max_engine_rpm = max_engine_rpm
        self.max_angular_velocity = max_angular_velocity

        self.count += 1

    def __del__(self):
        self.count -= 1


class QuadCopterController(QuadCopter):

    def __init__(self, angular_coordinates_desire, angle, angular_velocity, max_mixer_cmd=500):
        super().__init__(angle, angular_velocity)
        self.ax_des = angular_coordinates_desire[0]
        self.ay_des = angular_coordinates_desire[1]
        self.az_des = angular_coordinates_desire[2]
        self.max_mixer_cmd = max_mixer_cmd

        self.Pitch_cmd = 0
        self.Roll_cmd = 0
        self.Yaw_cmd = 0
        self.P_cmd = 0

        self.w = [0, 0, 0, 0]

    def Mixer(self):
        # mixing commands for quadcopter
        w0 = Saturation(self.P_cmd - self.Yaw_cmd + self.Roll_cmd + self.Pitch_cmd, self.max_engine_rpm, 0)
        w1 = Saturation(self.P_cmd + self.Yaw_cmd - self.Roll_cmd + self.Pitch_cmd, self.max_engine_rpm, 0)
        w2 = Saturation(self.P_cmd - self.Yaw_cmd - self.Roll_cmd - self.Pitch_cmd, self.max_engine_rpm, 0)
        w3 = Saturation(self.P_cmd + self.Yaw_cmd + self.Roll_cmd - self.Pitch_cmd, self.max_engine_rpm, 0)

        dw = 100

        dw0 = w0 - self.w[0]
        if dw0 > dw:
            dw0 = self.w[0] + dw
        elif dw0 < -dw:
            dw0 = self.w[0] - dw
        else:
            dw0 = w0

        dw1 = w1 - self.w[1]
        if dw1 > dw:
            dw1 = self.w[1] + dw
        elif dw1 < -dw:
            dw1 = self.w[1] - dw
        else:
            dw1 = w1

        dw2 = w2 - self.w[2]
        if dw2 > dw:
            dw2 = self.w[2] + dw
        elif dw2 < -dw:
            dw2 = self.w[2] - dw
        else:
            dw2 = w2

        dw3 = w3 - self.w[3]
        if dw3 > dw:
            dw3 = self.w[3] + dw
        elif dw3 < -dw:
            dw3 = self.w[3] - dw
        else:
            dw3 = w3

        self.w[0] = dw0
        self.w[1] = dw1
        self.w[2] = dw2
        self.w[3] = dw3


def Saturation(variable, max_val, min_val):
    if variable > max_val:
        return max_val
    elif variable < min_val:
        return min_val
    else:
        return variable


class PID:
    old_impact = 0

    def __init__(self, kp, kd, ki, set_point):
        self.set_point = set_point
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.old_impact = 0

    def setSetPoint(self, set_point):
        self.set_point = set_point

    def calculateOneStep(self, current_point, old_error, sum_error, dt):
        error = self.set_point - current_point
        impact = self.kp * error + self.kd * ((error - old_error) / dt) + self.ki * sum_error

        return impact


def PhysModel(P_des, ax_des, ay_des, angle_velocity_z_des, Timelapse, dt):
    motors_data = [[], [], [], []]
    angular_velocity_log = [[], [], []]
    angular_coordinate_log = [[], [], []]
    time = 0.0

    angular_coord = [ax_start, ay_start, az_start]
    angular_velocity = [wx_start, wy_start, wz_start]
    angular_coord_des = [ax_des, ay_des, angle_velocity_z_des]

    quad = QuadCopter(angular_coord, angular_velocity)
    quadControl = QuadCopterController(angular_coord_des, angular_coord, angular_velocity)

    wx_old_error = 0
    wx_sum_error = 0

    wy_old_error = 0
    wy_sum_error = 0

    wz_old_error = 0
    wz_sum_error = 0

    ax_old_error = 0
    ax_sum_error = 0

    ay_old_error = 0
    ay_sum_error = 0

    while time < Timelapse:
        angle_x_controller_pid = PID(10, 10, 0.5, quadControl.ax_des)
        angle_y_controller_pid = PID(10, 10, 0.5, quadControl.ay_des)

        angle_velocity_x_des = angle_x_controller_pid.calculateOneStep(quad.coordinate_x, ax_old_error, ax_sum_error,
                                                                       dt)
        angle_velocity_y_des = angle_y_controller_pid.calculateOneStep(quad.coordinate_y, ay_old_error, ay_sum_error,
                                                                       dt)

        velocity_x_controller = PID(50, 10, 0.2, angle_velocity_x_des)
        velocity_y_controller = PID(50, 10, 0.2, angle_velocity_y_des)
        velocity_z_controller = PID(50, 10, 10, angle_velocity_z_des)

        Pitch_cmd = velocity_x_controller.calculateOneStep(quad.velocity_x, wx_old_error, wx_sum_error, dt)
        Roll_cmd = velocity_y_controller.calculateOneStep(quad.velocity_y, wy_old_error, wy_sum_error, dt)
        Yaw_cmd = velocity_z_controller.calculateOneStep(quad.velocity_z, wz_old_error, wz_sum_error, dt)

        quadControl.Pitch_cmd = Saturation(Pitch_cmd, quadControl.max_mixer_cmd, -quadControl.max_mixer_cmd)
        quadControl.Roll_cmd = Saturation(Roll_cmd, quadControl.max_mixer_cmd, -quadControl.max_mixer_cmd)
        quadControl.Yaw_cmd = Saturation(Yaw_cmd, quadControl.max_mixer_cmd, -quadControl.max_mixer_cmd)
        quadControl.P_cmd = P_des

        quadControl.Mixer()

        M0 = ((quadControl.w[0] * pow(quad.throttle_ratio, 2)) * quad.leng / 2)  # calculate moment of every engine
        M1 = ((quadControl.w[1] * pow(quad.throttle_ratio, 2)) * quad.leng / 2)
        M2 = ((quadControl.w[2] * pow(quad.throttle_ratio, 2)) * quad.leng / 2)
        M3 = ((quadControl.w[3] * pow(quad.throttle_ratio, 2)) * quad.leng / 2)
        Mz = quad.z_inert_ratio * (pow(quadControl.w[3], 2) + pow(quadControl.w[1], 2) -
                                   pow(quadControl.w[0], 2) - pow(quadControl.w[2], 2))  # calculate gyro moment

        angular_acceleration_x = (M0 + M1 - M2 - M3) / quad.inert  # calculate X axis acceleration
        quad.velocity_x += angular_acceleration_x * dt  # calculate X axis angle speed
        wx_sum_error += (angle_velocity_x_des - quad.velocity_x)
        wx_old_error = angle_velocity_x_des - quad.velocity_x

        angular_acceleration_y = (M0 + M3 - M1 - M2) / quad.inert  # calculate Y axis acceleration
        quad.velocity_y += angular_acceleration_y * dt  # calculate Y axis angle speed
        wy_sum_error += (angle_velocity_y_des - quad.velocity_y)
        wy_old_error = angle_velocity_y_des - quad.velocity_y

        angular_acceleration_z = Mz / quad.inert  # calculate Z axis acceleration
        quad.velocity_z += angular_acceleration_z * dt  # calculate Z axis angle speed
        wz_sum_error += (angle_velocity_z_des - quad.velocity_z)
        wz_old_error = angle_velocity_z_des - quad.velocity_z

        quad.coordinate_x += quad.velocity_x * dt
        ax_sum_error += quadControl.ax_des - quad.coordinate_x
        ax_old_error = quadControl.ax_des - quad.coordinate_x

        quad.coordinate_y += quad.velocity_y * dt
        ay_sum_error += quadControl.ay_des - quad.coordinate_y
        ay_old_error = quadControl.ay_des - quad.coordinate_y

        quad.coordinate_z += quad.velocity_z * dt

        time += dt  # next time step

        motors_data[0].append(quadControl.w[0] * 10)  # write motors data
        motors_data[1].append(quadControl.w[1] * 10)
        motors_data[2].append(quadControl.w[2] * 10)
        motors_data[3].append(quadControl.w[3] * 10)

        angular_velocity_log[0].append(quad.velocity_x * 57.3)
        angular_velocity_log[1].append(quad.velocity_y * 57.3)
        angular_velocity_log[2].append(quad.velocity_z * 57.3)

        angular_coordinate_log[0].append(quad.coordinate_x * 57.3)
        angular_coordinate_log[1].append(quad.coordinate_y * 57.3)
        angular_coordinate_log[2].append(quad.coordinate_z * 57.3)

    return [angular_velocity_log, angular_coordinate_log,
            motors_data]  # this is proc output, that we will draw or analyse


if __name__ == '__main__':
    arr = PhysModel(P_des, 0.5, 0.4, 0, Timelapse, dt)

    Wx = np.array(arr[0][0])
    Wy = np.array(arr[0][1])
    Wz = np.array(arr[0][2])

    Ax = np.array(arr[1][0])
    Ay = np.array(arr[1][1])
    Az = np.array(arr[1][2])

    Motor1 = np.array(arr[2][0])
    Motor2 = np.array(arr[2][1])
    Motor3 = np.array(arr[2][2])
    Motor4 = np.array(arr[2][3])

    hrz = np.arange(0, Timelapse + dt, dt)

    pylab.figure(figsize=(7, 7), num='Angle speed controller')

    pylab.subplot(221)
    pylab.title("Angle speed", fontsize=12)
    pylab.xlabel("Time, sec", color="black")
    pylab.ylabel("Angle speed, deg/sec ", color="black")
    pylab.plot(hrz, Wx, hrz, Wy, hrz, Wz)
    pylab.legend(["Pitch", "Roll", "Yaw"], loc=1)

    pylab.subplot(222)
    pylab.title("Motors RPM", fontsize=12)
    pylab.xlabel("Time, sec")
    pylab.ylabel("RPM")
    pylab.plot(hrz, Motor1, hrz, Motor2, hrz, Motor3, hrz, Motor4)
    pylab.legend(["Motor 1", "Motor 2", "Motor 3", "Motor 4"], loc=1)

    pylab.subplot(223)
    pylab.title("Angle", fontsize=12)
    pylab.xlabel("Time, sec")
    pylab.ylabel("Angle, deg")
    pylab.plot(hrz, Ax, hrz, Ay, hrz, Az)
    pylab.legend(["Roll", "Pitch", "Yaw"], loc=1)

    pylab.tight_layout()
    pylab.show()

'''        Wx_input_parameters = [100, 0.001, 0.1, Wx_des, Wx_cur, Wx_old_error, Wx_sum_error, dt]
        Wx_impact = PID(Wx_input_parameters)  # calculate X axis impact
        Pitch_cmd = Saturation(Wx_impact, 500, 0)

        Wy_input_parameters = [100, 0.001, 0.1, Wy_des, Wy_cur, Wy_old_error, Wy_sum_error, dt]
        Wy_impact = PID(Wy_input_parameters)  # calculate Y axis impact
        Roll_cmd = Saturation(Wy_impact, 500, 0)

        Wz_input_parameters = [500, 1, 1, Wz_des, Wz_cur, Wz_old_error, Wz_sum_error, dt]
        Wz_impact = PID(Wz_input_parameters)  # calculate Z axis impact
        Yaw_cmd = Saturation(Wz_impact, 10, -10)
        
        def PID(pid_input_parameters):  # [kp,ki,kd, setpoint, currentpoint, old_error, sum_error,dt]

    # variables definition
    kp = pid_input_parameters[0]  # PID Proportional parameter
    ki = pid_input_parameters[1]  # PID Integral parameter
    kd = pid_input_parameters[2]  # PID derivative parameter
    set_point = pid_input_parameters[3]
    current_point = pid_input_parameters[4]
    old_error = pid_input_parameters[5]
    sum_error = pid_input_parameters[6]
    dt = pid_input_parameters[7]

    # calculate PID impact
    error = set_point - current_point
    impact = kp * error + kd * ((error - old_error) / dt) + ki * sum_error

    return impact
'''
