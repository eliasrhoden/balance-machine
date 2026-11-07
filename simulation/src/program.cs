using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using Siemens.Simatic.Simulation.Runtime;

public class PLCInstance
{
    public IInstance instance { get; set; }
    private bool IsConfigured { get; set; }

    private SimotionServoDrive servo;
    private ulong bustime_us = 0;

    public ulong sync_point_counter = 0;
    public double avg_sync_time_virtual = 4.0;
    public double avg_sync_time_real = 4.0;
    public double avg_read_plc_time_ms = 0.0;
    public double avg_servo_step_time_ms = 0.0;
    public double avg_write_plc_time_ms = 0.0;

    private double last_real_sync_time_ms = 0.0;
    private readonly Stopwatch simulationClock = new Stopwatch();

    private PhysicSim sim;

    public PLCInstance(PhysicSim simulation, string instanceName = "GlennLogic222")
    {
        sim = simulation;
        // bin/Debug/net48 -> project root, mirrors PhysicSim.StartStudio's mujoco_runtime lookup.
        string projectRoot = Directory.GetParent(AppDomain.CurrentDomain.BaseDirectory).Parent.Parent.Parent.FullName;
        string fmuPath = Path.Combine(projectRoot, "simotion FMU", "DriveSim_Designer_CODE_v2202", "DriveSimDesigner_SERVO_double.fmu");
        servo = new SimotionServoDrive(fmuPath);

        var info = SimulationRuntimeManager.RegisteredInstanceInfo
            .FirstOrDefault(i => i.Name == instanceName);

        if (string.IsNullOrEmpty(info.Name))
        {
            throw new InvalidOperationException($"Failed to find PLC instance '{instanceName}' in PLCSIM Advanced.");
        }

        instance = SimulationRuntimeManager.CreateInterface(instanceName);
        IsConfigured = false;
        Update();

        foreach (var tag in instance.TagInfos)
        {
            Console.WriteLine($"{tag.Name} : {tag.DataType}");
        }


        instance.OnSoftwareConfigurationChanged += instance_OnSoftwareConfigurationChanged;
        instance.OnSyncPointReached += instance_OnSyncPointReached;
        instance.IsSendSyncEventInDefaultModeEnabled = true;

        instance.OperatingMode = EOperatingMode.SingleStep_Bus;
        simulationClock.Start();
        try
        {
            instance.RunToNextSyncPoint();
        }catch(Exception e)
        {
            
        }

        instance.RunToNextSyncPoint(100, 0);
    }

    /// <summary>Throws if OnSyncPointReached can never fire: it requires either a synchronized/single-step
    /// operating mode, or freerun mode with IsSendSyncEventInDefaultModeEnabled explicitly set to true.</summary>
    private void CheckSyncPointConfiguration()
    {
        Console.WriteLine($"[SyncCheck] OperatingState={instance.OperatingState}, OperatingMode={instance.OperatingMode}, " +
            $"IsSendSyncEentInDefaultModeEnabled={instance.IsSendSyncEventInDefaultModeEnabled}");

        // Synchronized modes sit in Freeze until the host drives them via RunToNextSyncPoint(); that's expected, not an error.
        bool isSynchronizedMode = instance.OperatingMode != EOperatingMode.Default;
        if (instance.OperatingState != EOperatingState.Run &&
            !(isSynchronizedMode && instance.OperatingState == EOperatingState.Freeze))
        {
            throw new InvalidOperationException($"Instance is not in Run state ({instance.OperatingState}); no sync points can occur.");
        }

        if (instance.OperatingMode == EOperatingMode.Default && !instance.IsSendSyncEventInDefaultModeEnabled)
        {
            throw new InvalidOperationException("Instance is running freerun (OperatingMode=Default) with " +
                "IsSendSyncEventInDefaultModeEnabled=false. OnSyncPointReached will never fire in this configuration. " +
                "Either set instance.IsSendSyncEventInDefaultModeEnabled = true, or switch OperatingMode to a " +
                "synchronized/single-step mode (e.g. TimespanSynchronized_C).");
        }
    }

    private void instance_OnSyncPointReached(IInstance in_Sender, ERuntimeErrorCode in_ErrorCode,
        DateTime in_DateTime, uint in_Id, long in_TimeSinceSameSyncPoint_ns, long in_TimeSinceAnySyncPoint_ns, uint in_SyncPointCount)
    {
        //Console.WriteLine($"sync ID:{in_Id} time since same sync: {in_TimeSinceSameSyncPoint_ns/1000000} time since sny sync: {in_TimeSinceAnySyncPoint_ns/1000000}");

        const double ALFA = 0.99;

        if (in_Id == 100)
        {
            bustime_us += (ulong)(in_TimeSinceSameSyncPoint_ns / 1000);

            avg_sync_time_virtual = avg_sync_time_virtual*ALFA +  (1-ALFA)*(in_TimeSinceSameSyncPoint_ns/1000000.0);

            double current_sync_real_time_ms = simulationClock.Elapsed.TotalMilliseconds;

            avg_sync_time_real = avg_sync_time_real*ALFA +  (1-ALFA)*(current_sync_real_time_ms - last_real_sync_time_ms);

            last_real_sync_time_ms = current_sync_real_time_ms;
            try
            {
                Stopwatch operationClock = Stopwatch.StartNew();

                var qTelegram = read105FromPlcInstance();
                operationClock.Stop();
                avg_read_plc_time_ms = avg_read_plc_time_ms * ALFA + (1 - ALFA) * operationClock.Elapsed.TotalMilliseconds;

                operationClock.Restart();
                var iTelegram = servo.Step(qTelegram, bustime_us / 1000000);
                operationClock.Stop();
                avg_servo_step_time_ms = avg_servo_step_time_ms * ALFA + (1 - ALFA) * operationClock.Elapsed.TotalMilliseconds;

                var simOut = sim.Step(servo.velocity, servo.position, avg_sync_time_virtual/1000.0);

                writeMeasurementsToPlcInstance(simOut);

                operationClock.Restart();
                write105ToPlcInstance(iTelegram);
                operationClock.Stop();
                avg_write_plc_time_ms = avg_write_plc_time_ms * ALFA + (1 - ALFA) * operationClock.Elapsed.TotalMilliseconds;

                sync_point_counter++;
            }
            catch (Exception e)
            {
                Console.WriteLine("ERROR");
                Console.WriteLine(e.ToString());
                Update();
                //remainingSyncPoints = 0;

            }

        }
        try
        {
            instance.RunToNextSyncPoint(in_Id, bustime_us * 1000);
        }catch(Exception e)
        {
            Console.WriteLine($"Error when calling RunToNextSync, ID:{in_Id}");
            throw e;
        }
        
    }

    /// <summary>Writes the 4 PinballMeasurement.* floats in one batched WriteSignals call instead of 4 separate WriteFloat calls.</summary>
    private void writeMeasurementsToPlcInstance(SimOutput simOut)
    {
        const string tag = "PinballSignals";

        float ballPos_m = (float)simOut.ballPosition_m;

        ballPos_m = (float) Math.Round(ballPos_m * 100.0)/100.0f;

        ballPos_m = -(ballPos_m -0.47f);

        var signals = new[]
        {
            MakeFloatWriteSignal($"{tag}.ballPosition", ballPos_m),

            //MakeFloatWriteSignal($"{tag}.servoPos", (float)simOut.servo_position_deg),
            //MakeFloatWriteSignal($"{tag}.servoVel", (float)simOut.servo_angle_deg_ps),
            //MakeFloatWriteSignal($"{tag}.ballPosition", (float)simOut.ballPosition_m),
            //MakeFloatWriteSignal($"{tag}.velocity_mps", (float)simOut.ballVelocity_mps),
        };

        instance.WriteSignals(ref signals);

        foreach (var signal in signals)
            CheckSignalError(signal);
    }

    private void instance_OnSoftwareConfigurationChanged(IInstance in_Sender, SOnSoftwareConfigChangedParameter event_param)
    {
        Console.WriteLine("CONFIG UPDATED");
        Update();
    }

    public void Update()
    {
        instance.UpdateTagList(ETagListDetails.IODB);
        IsConfigured = true;
    }

    /// <summary>Reads the PLC's outgoing control telegram (GlennAxisTelegramOut.*), i.e. what the FMU should receive as input.
    /// Uses one batched ReadSignals call instead of one PLCSIM API call per tag.</summary>
    public Telegram105Ctrl read105FromPlcInstance()
    {
        const string tel = "GlennAxisTelegramOut";

        var names = new List<string>();
        AddBitTagNames(names, tel, "STW1", Stw1BitNames);
        AddBitTagNames(names, tel, "STW2", Stw2BitNames);
        AddBitTagNames(names, tel, "G1_STW", G1StwBitNames);
        names.Add($"{tel}.NSOLL_B");
        names.Add($"{tel}.MOMRED");
        names.Add($"{tel}.XERR");
        names.Add($"{tel}.KPC");

        var signals = new SDataValueByName[names.Count];
        for (int i = 0; i < names.Count; i++)
            signals[i].Name = names[i];

        instance.ReadSignals(ref signals);

        int index = 0;
        return new Telegram105Ctrl
        {
            STW1 = UnpackBits(signals, ref index, Stw1BitNames.Length),
            STW2 = UnpackBits(signals, ref index, Stw2BitNames.Length),
            G1_STW = UnpackBits(signals, ref index, G1StwBitNames.Length),
            NSOLL_B = ReadScalarInt32(signals, ref index),
            MOMRED = ReadScalarUInt16(signals, ref index),
            XERR = ReadScalarInt32(signals, ref index),
            KPC = ReadScalarInt32(signals, ref index),
        };
    }

    /// <summary>Writes the drive's status telegram (into GlennAxisTelegramIn.*) as reported by the FMU.
    /// Uses one batched WriteSignals call instead of one PLCSIM API call per tag.</summary>
    public void write105ToPlcInstance(Telegram105Status telegram)
    {
        const string tel = "GlennAxisTelegramIn";

        var signalList = new List<SDataValueByName>();
        AddBitWriteSignals(signalList, tel, "ZSW1", Zsw1BitNames, telegram.ZSW1);
        AddBitWriteSignals(signalList, tel, "ZSW2", Zsw2BitNames, telegram.ZSW2);
        AddBitWriteSignals(signalList, tel, "MELDW", MeldwBitNames, telegram.MELDW);
        AddBitWriteSignals(signalList, tel, "G1_ZSW", G1_Zsw1BitNames, telegram.G1_ZSW);
        signalList.Add(MakeUInt32WriteSignal($"{tel}.NIST_B", unchecked((uint)telegram.NIST_B)));
        signalList.Add(MakeUInt32WriteSignal($"{tel}.G1_XIST1", unchecked((uint)telegram.G1_XIST1)));
        signalList.Add(MakeUInt32WriteSignal($"{tel}.G1_XIST2", unchecked((uint)telegram.G1_XIST2)));

        var signals = signalList.ToArray();
        instance.WriteSignals(ref signals);

        foreach (var signal in signals)
            CheckSignalError(signal);
    }

    // Bit order (LSB first) matches the PLC struct layout, see readme.md.
    private static readonly string[] Stw1BitNames =
    {
        "Reserved_Bit08", "EnableESRReaction", "ControlByPlc", "RampFunctionGeneratorActive",
        "OpenHoldingBrake", "Reserved_Bit13", "ClosedLoopTorqueControl", "Reserved_Bit15",
        "On", "NoCoastStop", "NoQuickStop", "EnableOperation",
        "EnableRampGenerator", "UnfreezeRampGenerator", "EnableSetpoint", "FaultAcknowledge",
    };

    private static readonly string[] G1StwBitNames =
    {
        "Reserved_Bit08", "Reserved_Bit09", "Reserved_Bit10", "Reserved_Bit11",
        "Reserved_Bit12", "AbsoluteValueCyclically", "RequestParkingEncoder", "AcknowledgeError",
        "Function1Request", "Function2Request", "Function3Request", "Function4Request",
        "Command0Request", "Command1Request", "Command2Request", "Mode",
    };

    private static readonly string[] Stw2BitNames =
    {
        "TravelToFixedEndstop", "DriveDataSetSelectionBit3", "DriveDataSetSelectionBit4", "MotorSwitchoverFinished",
        "MasterSignOfLifeBit0", "MasterSignOfLifeBit1", "MasterSignOfLifeBit2", "MasterSignOfLifeBit3",
        "DriveDataSetSelectionBit0", "DriveDataSetSelectionBit1", "DriveDataSetSelectionBit2", "Reserved_Bit03",
        "BypassRampFunctionGenerator", "Reserved_Bit05", "IntegratorInhibit", "ParkingAxisSelection",
    };

    private static readonly string[] Zsw1BitNames =
    {
        "NoSpeedDeviation", "ControlRequested", "SpeedComparisonValusReachedExeeded", "AlarmClassBit0",
        "AlarmClassBit1", "Reserved_Bit13", "ClosedLoopTorqueControlActive", "Reserved_Bit15",
        "ReadyToSwitchOn", "ReadyToOperate", "OperationEnabled", "FaultPresent",
        "NoCoastStopActivated", "NoQuickStopActivated", "SwitchingOnInhibited", "AlarmPresent",
    };

    private static readonly string[] G1_Zsw1BitNames =
    {
        "Probe1Deflected", "Probe2Deflected", "Reserved_Bit10", "EncoderFaultAcknowledgeActive",
        "HomePositionExecuted", "AbsoluteValueCyclicallyExecuted", "ParkingSensorExecuted", "SensorError",
        "Function1Active", "Function2Active", "Function3Active", "Function4Active",
        "Value1Available", "Value2Available", "Value3Available", "Value4Available",
    };

    private static readonly string[] Zsw2BitNames =
    {
        "TravelToFixedEndStopActive", "DriveDataSetEffectiveBit3", "DriveDataSetEffectiveBit4", "MotorDataSetChangeoverActive",
        "SlaveSignOfLifeBit0", "SlaveSignOfLifeBit1", "SlaveSignOfLifeBit2", "SlaveSignOfLifeBit3",
        "DriveDataSetEffectiveBit0", "DriveDataSetEffectiveBit1", "DriveDataSetEffectiveBit2", "Reserved_Bit03",
        "RampFunctionGeneratorInactive", "OpenHoldingBrake", "IntegratorInhibit", "ParkingAxisActive",
    };

    private static readonly string[] MeldwBitNames =
    {
        "SpeedTolerance", "Reserved_Bit09", "Reserved_Bit10", "ControllerEnable",
        "DriveReady", "PulsesEnabled", "Reserved_Bit14", "Reserved_Bit15",
        "RampUpDownCompleted", "TorqueUtilizationOk", "ActSpeedValue3Ok", "ActSpeedValue2Ok",
        "Reserved_Bit04", "VariableFunctionality", "NoWarningMotorTemperature", "NoPowerUnitOvertemperature",
    };

    private static void AddBitTagNames(List<string> names, string tel, string field, string[] bitNames)
    {
        foreach (var bitName in bitNames)
            names.Add($"{tel}.{field}.{bitName}");
    }

    private static void CheckSignalError(SDataValueByName signal)
    {
        if (signal.ErrorCode != ERuntimeErrorCode.OK)
            throw new InvalidOperationException($"PLCSIM signal error for '{signal.Name}': {signal.ErrorCode}");
    }

    private static ushort UnpackBits(SDataValueByName[] signals, ref int index, int count)
    {
        ushort value = 0;
        for (int i = 0; i < count; i++)
        {
            var signal = signals[index++];
            CheckSignalError(signal);
            if (signal.DataValue.Bool)
                value |= (ushort)(1 << i);
        }
        // Swap lower byte with upper byte
        return (ushort)(((value & 0xFF) << 8) | ((value >> 8) & 0xFF));
    }

    private static int ReadScalarInt32(SDataValueByName[] signals, ref int index)
    {
        var signal = signals[index++];
        CheckSignalError(signal);
        return signal.DataValue.Int32;
    }

    private static ushort ReadScalarUInt16(SDataValueByName[] signals, ref int index)
    {
        var signal = signals[index++];
        CheckSignalError(signal);
        return signal.DataValue.UInt16;
    }

    private static void AddBitWriteSignals(List<SDataValueByName> signals, string tel, string field, string[] bitNames, ushort value)
    {
        // Swap lower byte with upper byte
        value = (ushort)(((value & 0xFF) << 8) | ((value >> 8) & 0xFF));
        for (int i = 0; i < bitNames.Length; i++)
        {
            signals.Add(new SDataValueByName
            {
                Name = $"{tel}.{field}.{bitNames[i]}",
                DataValue = new SDataValue { Type = EPrimitiveDataType.Bool, Bool = (value & (1 << i)) != 0 },
            });
        }
    }

    private static SDataValueByName MakeUInt32WriteSignal(string name, uint value)
    {
        return new SDataValueByName
        {
            Name = name,
            DataValue = new SDataValue { Type = EPrimitiveDataType.UInt32, UInt32 = value },
        };
    }

    private static SDataValueByName MakeFloatWriteSignal(string name, float value)
    {
        return new SDataValueByName
        {
            Name = name,
            DataValue = new SDataValue { Type = EPrimitiveDataType.Float, Float = value },
        };
    }
}

public static class Program
{
    public static void Main()
    {
        // bin/Debug/net48 -> project root, mirrors PhysicSim.StartStudio's mujoco_runtime lookup.
        string projectRoot = Directory.GetParent(AppDomain.CurrentDomain.BaseDirectory).Parent.Parent.Parent.FullName;
        string modelPath = Path.Combine(projectRoot, "..", "mujoco-robot", "scene.xml");

        var sim = new PhysicSim(modelPath);
        Console.WriteLine("Mujoco started");
        var plc = new PLCInstance(sim);
        
        Console.WriteLine("PLC instance ready.");

        while (true)
        {

            Thread.Sleep(5000);
            Console.WriteLine($"Nr syncs: {plc.sync_point_counter} Virtual CT: {plc.avg_sync_time_virtual:F1}ms Real CT: {plc.avg_sync_time_real:F1}ms " +
                $"Read PLC: {plc.avg_read_plc_time_ms:F1}ms Servo.Step: {plc.avg_servo_step_time_ms:F1}ms Write PLC: {plc.avg_write_plc_time_ms:F1}ms");
            plc.sync_point_counter = 0;

        }

    }
}