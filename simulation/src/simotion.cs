using System;

/// <summary>
/// Wraps the DriveSimDesigner_SERVO FMU (a Simotion servo drive) behind the PROFIdrive
/// Telegram 105 control/status interface.
/// </summary>
public sealed class SimotionServoDrive : IDisposable
{
    private readonly FmuCoSimulation _fmu;
    private readonly double _profidriveTs;
    private double simTime;

    public double position = 0.0;
    public double velocity = 0.0;

    public SimotionServoDrive(string fmuPath, string parametersPath = "DriveSimDesigner_SERVO.Parameters.xml",
        double startTime = 0, string instanceName = "SimotionServo", bool loggingEnabled = false)
    {
        simTime = startTime;
        _fmu = new FmuCoSimulation(fmuPath, instanceName, loggingEnabled);
        _fmu.SetupExperiment(startTime);
        Console.WriteLine("[SimotionServoDrive] EnterInitializationMode"); Console.Out.Flush();
        _fmu.EnterInitializationMode();
        FmuParameterSet.Load(parametersPath).ApplyTo(_fmu);
        Console.WriteLine("[SimotionServoDrive] ExitInitializationMode"); Console.Out.Flush();
        _fmu.ExitInitializationMode();
        Console.WriteLine("[SimotionServoDrive] ExitInitializationMode done"); Console.Out.Flush();

        // p2048_IF1_PZD_t_samp is in ms.
        _profidriveTs = _fmu.GetReal("Param.Communication.p2048_IF1_PZD_t_samp") / 1000.0;
    }

    /// <summary>Writes the control telegram, advances the FMU by one step, and returns the resulting status telegram.</summary>
    public Telegram105Status Step(Telegram105Ctrl telegramCtrl, double currentTime_s)
    {
        bool okToDiscardOldStates = true;
        // Timestep depends if we use our own mechanical model, here we use the internal
        // thus we need to run it below the profidrive sampling time 'Param.Communication.p2048_IF1_PZD_t_samp'

        // NOTE: This only works when Config.ExternalLoad == 0
        /*
        int DIVID = 4;
        double simotionTimestep = (_profidriveTs)/DIVID; // Evenly dividable
        Console.WriteLine($"SIMOTION t={currentTime}");
        */
        double simotionTimestep_s = (_profidriveTs); // Evenly dividable
        
        WriteTelegram(telegramCtrl);
        _fmu.DoStep(currentTime_s, simotionTimestep_s, okToDiscardOldStates);
        //_fmu.DoStep(simTime, simotionTimestep_s, okToDiscardOldStates);

        velocity = _fmu.GetReal("n_act_intLoad") * 360.0;//2*Math.PI ;
        position = _fmu.GetReal("x_act_intLoad") * 360.0;//2*Math.PI;
        return ReadTelegram();
    }

    private void WriteTelegram(Telegram105Ctrl telegram)
    {

        _fmu.SetReal("STW1", telegram.STW1);
        _fmu.SetReal("STW2", telegram.STW2);
        _fmu.SetReal("NSOLL_B", telegram.NSOLL_B);
        _fmu.SetReal("G1_STW", telegram.G1_STW);
        _fmu.SetReal("MOMRED", telegram.MOMRED);
        _fmu.SetReal("XERR", telegram.XERR);
        _fmu.SetReal("KPC", telegram.KPC);
        _fmu.SetReal("S_STW1", telegram.S_STW1);
        _fmu.SetReal("S_STW2", telegram.S_STW2);
    }

    private Telegram105Status ReadTelegram()
    {
        return new Telegram105Status
        {
            ZSW1 = ReadInt16("ZSW1"),
            ZSW2 = ReadInt16("ZSW2"),
            NIST_B = ReadInt32("NIST_B"),
            G1_ZSW = ReadInt16("G1_ZSW"),
            G1_XIST1 = ReadInt32("G1_XIST1"),
            G1_XIST2 = ReadInt32("G1_XIST2"),
            MELDW = ReadInt16("MELDW"),
            S_ZSW1 = ReadInt16("S_ZSW1"),
        };
    }

    private UInt16 ReadInt16(string variableName)
    {
        
        double value = _fmu.GetReal(variableName);
        if (value != Math.Truncate(value) || value < UInt16.MinValue || value > UInt16.MaxValue)
        {
            throw new InvalidOperationException($"FMU variable '{variableName}' has value {value}, which cannot be represented as Int16.");
        }

        return checked((UInt16) value);
    }

    private Int32 ReadInt32(string variableName)
    {
        double value = _fmu.GetReal(variableName);
        if (value != Math.Truncate(value) || value < Int32.MinValue || value > Int32.MaxValue)
        {
            throw new InvalidOperationException($"FMU variable '{variableName}' has value {value}, which cannot be represented as Int32.");
        }
        return checked((Int32)value);
    }

    public void Dispose() => _fmu.Dispose();
}
