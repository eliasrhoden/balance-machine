using System;
using System.Globalization;
using System.IO;
using System.Text;
using Mujoco;




public class SimOutput
{
	public double ballPosition_m;
	public double ballVelocity_mps;
	public double servo_position_deg;
	public double servo_angle_deg_ps;
}

public unsafe class PhysicSim
{

	private MujocoLib.mjModel_* model;
	private MujocoLib.mjData_* data;
	private readonly object dataLock = new object();
	private PassiveViewer viewer;
	private int servoIndx ,ballIndx;

	int servoJoint; 
	int ballJoint;
	int servo_actuator; 

	public PhysicSim(string modelPath)
	{
		StringBuilder error = new StringBuilder(1024);
		model = MujocoLib.mj_loadXML(Path.GetFullPath(modelPath), null, error, error.Capacity);
		if (model == null)
		{
			throw new InvalidOperationException($"MuJoCo could not load the model: {error}");
		}

		data = MujocoLib.mj_makeData(model);
		if (data == null)
		{
			MujocoLib.mj_deleteModel(model);
			throw new InvalidOperationException("MuJoCo could not allocate simulation data.");
		}

		servoJoint = FindObject(model, MujocoLib.mjtObj.mjOBJ_JOINT, "servo");
		ballJoint = FindObject(model, MujocoLib.mjtObj.mjOBJ_JOINT, "slide");
		servo_actuator = FindObject(model, MujocoLib.mjtObj.mjOBJ_ACTUATOR, "servo_actuator");

		servoIndx = model->jnt_qposadr[servoJoint];
		ballIndx = model->jnt_qposadr[ballJoint];

		// board_hinge's "ref" in the XML sets qpos0, so mj_makeData already starts at that pose.
		MujocoLib.mj_forward(model,data);

		viewer = new PassiveViewer(model, data, dataLock);
	}


	public SimOutput Step(double servoVelocity_deg_s, double servoPos_deg, double Ts)
	{
		
		SimOutput res = new SimOutput();
		try
		{

			lock (dataLock)
			{
				model->opt.timestep = Ts;

				// Check how the servo actuator is defined in the robot.xml
				//data->ctrl[servo_actuator] = servoVelocity_deg_s * (Math.PI / 180.0);
				data->ctrl[servo_actuator] = servoPos_deg * (Math.PI / 180.0);

				MujocoLib.mj_step(model, data);

				res.ballPosition_m = data->qpos[ballIndx];
				res.ballVelocity_mps = data->qvel[ballIndx];

				res.servo_position_deg = data->qpos[servoIndx]*(180.0/Math.PI);
				res.servo_angle_deg_ps = data->qvel[servoIndx]*(180.0/Math.PI);
			}

		}
		finally
		{

		}

		return res;
	} 


	public void deleteRefs()
	{
		try
		{
			viewer?.Dispose();
			MujocoLib.mj_deleteData(data);
			MujocoLib.mj_deleteModel(model);
		}
		catch 
		{
			
		}
	}

	private static int FindObject(MujocoLib.mjModel_* model, MujocoLib.mjtObj type, string name)
	{
		int id = MujocoLib.mj_name2id(model, (int)type, name);
		if (id < 0)
		{
			throw new ArgumentException($"{type} '{name}' was not found.");
		}

		return id;
	}
}
