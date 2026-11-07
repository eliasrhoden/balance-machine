import time

import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt 
import traj


SERVO_POS_SETPOINTS = []
SERVO_ACT_VELOCITY = []
SERVO_ACT_POS = []
BALL_VELOCITYS = []
BALL_VELOCITYS_SETP = []
BALL_POSITIONS = []
BALL_POSITIONS_SETP = []
TIMES = []
SERVO_TORQUE = []



m = mujoco.MjModel.from_xml_path(r"../mujoco-robot\scene.xml")
d:mujoco.MjData = mujoco.MjData(m)

trajGen = traj.Traj(0.2,0.7)


with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()


    servo_joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "servo")
    ball_joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "ball")
    servo_actuator_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_actuator")

    servo_qpos_adr = m.jnt_qposadr[servo_joint_id]
    ball_qpos_adr = m.jnt_qposadr[ball_joint_id]
    print("ball_addr",ball_qpos_adr)

    viewer.cam.azimuth = 180

    q = np.zeros(m.nq)
    # Starting pos
    q[ball_qpos_adr] = 0.0
    q[servo_qpos_adr] = np.deg2rad(0)

    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    viewer.sync()


    Ts = m.opt.timestep
    print("Ts",m.opt.timestep)

    error_integral = 0

    t0 = time.time()
    while viewer.is_running():
        step_start = time.time()

        pinball_pos_m = -d.qpos[ball_qpos_adr]
        pinball_vel_mps = -d.qvel[ball_qpos_adr]

        board_angle = d.qpos[servo_joint_id] - np.pi

        MAX_VEL = 0.4

        t = time.time() - t0

        pos_setp = 0.0

        PER_TIME = 20
        if t > 2:
            if(np.mod(t,PER_TIME) < PER_TIME/2):
                pos_setp = -0.3
            else:
                pos_setp = 0.3


        trajGen.step(pos_setp,Ts=Ts)
        pos_setp_smooth = trajGen.p

        ball_vel_setp = trajGen.v

        pos_error = (pos_setp_smooth - pinball_pos_m)
        vel_error = (ball_vel_setp - pinball_vel_mps)
        error_integral += pos_error*Ts
        #1.1721002 , 0.48912276, 0.3162
        servo_pos_setp = 1.1*pos_error + 0.5*vel_error + 0.3*error_integral
        servo_pos_setp = -1*servo_pos_setp

        servo_pos_setp = np.clip(servo_pos_setp,np.deg2rad(-30),np.deg2rad(30))

        d.ctrl[servo_actuator_id] = servo_pos_setp

        SERVO_POS_SETPOINTS.append(servo_pos_setp)
        SERVO_ACT_VELOCITY.append(d.qvel[servo_joint_id]*60/(2*np.pi))
        SERVO_ACT_POS.append(d.qpos[servo_joint_id])
        BALL_POSITIONS.append(pinball_pos_m)
        BALL_VELOCITYS.append(pinball_vel_mps)
        TIMES.append(time.time())
        BALL_VELOCITYS_SETP.append(ball_vel_setp)
        BALL_POSITIONS_SETP.append(pos_setp_smooth)

        mujoco.mj_step(m, d)
        # For a velocity actuator, d.ctrl stores the target velocity command.
        # The actual actuator torque applied to the joint is in qfrc_actuator.
        SERVO_TORQUE.append(d.qfrc_actuator[servo_joint_id])
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)



TIMES = np.array(TIMES)
TIMES -= TIMES[0]

#data = np.array([TIMES,BALL_VELOCITYS,SERVO_POS_SETPOINTS,SERVO_ACT_VELOCITY,BALL_POSITIONS])
#np.save('glenn.npy',data)

plt.figure()
plt.plot(TIMES,SERVO_POS_SETPOINTS,'k--')
plt.plot(TIMES,SERVO_ACT_POS)
plt.title("Servo position")

plt.figure()
plt.plot(TIMES,BALL_VELOCITYS_SETP,'k--')
plt.plot(TIMES,BALL_VELOCITYS)
plt.title('Ball velocity')

plt.figure()
plt.plot(TIMES,BALL_POSITIONS_SETP,'k--')
plt.plot(TIMES,BALL_POSITIONS)
plt.title('Ball position')

plt.figure()
plt.plot(TIMES,SERVO_TORQUE)
plt.ylabel("Servo Torque [Nm]")
plt.xlabel("Time [s]")

plt.show()


        