using System;
using System.IO;

/// <summary>
/// Plain data holder for the PROFIdrive "Standard Telegram 105" PZD words.
/// Field names match the FMU's telegram signals 1:1 (see DriveSimDesigner_SERVO.fmu ModelVariables, VR 1-14 and 24-37).
/// </summary>
public sealed class Telegram105Ctrl
{
    // Control telegram: PLC -> Drive (these become FmuCoSimulation inputs)
    public UInt16 STW1;
    public UInt16 STW2;
    public Int32 NSOLL_B;
    public UInt16 G1_STW;
    public UInt16 MOMRED;
    public Int32 XERR;
    public Int32 KPC;
    public UInt16 S_STW1;
    public UInt16 S_STW2;

    public override string ToString()
    {
        return $" CTRL TELEGRAM STW1: {STW1} G1_STW {G1_STW}";
    }
}


public sealed class Telegram105Status
{

    // Status telegram: Drive -> PLC (these come from FmuCoSimulation outputs)
    public UInt16 ZSW1;
    public UInt16 ZSW2;
    public Int32 NIST_B;
    public UInt16 G1_ZSW;
    public Int32 G1_XIST1;
    public Int32 G1_XIST2;
    public UInt16 MELDW;
    public UInt16 S_ZSW1;

    // ZSW2 SlaveSignOfLifeBit0..3 occupy bits 12-15 after the byte swap applied by ReadBitsAsUInt16.
    public static int SignOfLife(UInt16 zsw2) => (zsw2 >> 12) & 0xF;

    public override string ToString()
    {
        return $"STATUS TELEGRAM ZSTW1: {ZSW1} G1_ZSTW {G1_ZSW} SOL {SignOfLife(ZSW2)}";
    }
}
