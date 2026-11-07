# S210 simulation

This application interacts with
* PLCSimAdvanced V8.1 (That runs the User PLC Program, i.e. the logic we want to test)
* Simotion DriveSimDesignerServo FMU, a FMU simulating the S210 servo including logic/sequence control
* Mujoco simulator, for the physics simulation that the S210 servo interacts with


## Big picture
PLCSim advanced runs the PLC Application, this application reads/writes the drive telegram (Siemens Profidrive telegram 105) to the FMU unit, that outputs a velocity to the Mujoco simulation.

## Profidrive Telegram 105

Struct in PLCSim for read/write

```
GlennAxisTelegramIn : Struct
GlennAxisTelegramIn.ZSW1 : Struct
GlennAxisTelegramIn.NIST_B : DWord
GlennAxisTelegramIn.ZSW2 : Struct
GlennAxisTelegramIn.MELDW : Struct
GlennAxisTelegramIn.G1_ZSW : Struct
GlennAxisTelegramIn.G1_XIST1 : DWord
GlennAxisTelegramIn.G1_XIST2 : DWord

GlennAxisTelegramOut : Struct
GlennAxisTelegramOut.STW1 : Struct
GlennAxisTelegramOut.NSOLL_B : DWord
GlennAxisTelegramOut.STW2 : Struct
GlennAxisTelegramOut.MOMRED : Word
GlennAxisTelegramOut.G1_STW : Struct
GlennAxisTelegramOut.XERR : DWord
GlennAxisTelegramOut.KPC : DWord
```


### Control words (`GlennAxisTelegramOut`, PLC → drive)

#### STW1 (basic control word)
| Offset | Name |
|--------|------|
| x.0 | Reserved_Bit08 |
| x.1 | EnableESRReaction |
| x.2 | ControlByPlc |
| x.3 | RampFunctionGeneratorActive |
| x.4 | OpenHoldingBrake |
| x.5 | Reserved_Bit13 |
| x.6 | ClosedLoopTorqueControl |
| x.7 | Reserved_Bit15 |
| (x+1).0 | On |
| (x+1).1 | NoCoastStop |
| (x+1).2 | NoQuickStop |
| (x+1).3 | EnableOperation |
| (x+1).4 | EnableRampGenerator |
| (x+1).5 | UnfreezeRampGenerator |
| (x+1).6 | EnableSetpoint |
| (x+1).7 | FaultAcknowledge |

#### STW2 (extended control word)
| Offset | Name |
|--------|------|
| x.0 | TravelToFixedEndstop |
| x.1 | DriveDataSetSelectionBit3 |
| x.2 | DriveDataSetSelectionBit4 |
| x.3 | MotorSwitchoverFinished |
| x.4 | MasterSignOfLifeBit0 |
| x.5 | MasterSignOfLifeBit1 |
| x.6 | MasterSignOfLifeBit2 |
| x.7 | MasterSignOfLifeBit3 |
| (x+1).0 | DriveDataSetSelectionBit0 |
| (x+1).1 | DriveDataSetSelectionBit1 |
| (x+1).2 | DriveDataSetSelectionBit2 |
| (x+1).3 | Reserved_Bit03 |
| (x+1).4 | BypassRampFunctionGenerator |
| (x+1).5 | Reserved_Bit05 |
| (x+1).6 | IntegratorInhibit |
| (x+1).7 | ParkingAxisSelection |

#### G1_STW (encoder 1 control word)
| Offset | Name |
|--------|------|
| x.0 | Reserved_Bit08 |
| x.1 | Reserved_Bit09 |
| x.2 | Reserved_Bit10 |
| x.3 | Reserved_Bit11 |
| x.4 | Reserved_Bit12 |
| x.5 | AbsoluteValueCyclically |
| x.6 | RequestParkingEncoder |
| x.7 | AcknowledgeError |
| (x+1).0 | Function1Request |
| (x+1).1 | Function2Request |
| (x+1).2 | Function3Request |
| (x+1).3 | Function4Request |
| (x+1).4 | Command0Request |
| (x+1).5 | Command1Request |
| (x+1).6 | Command2Request |
| (x+1).7 | Mode |

### Status words (`GlennAxisTelegramIn`, drive → PLC)

#### ZSW1 (basic status word)
| Offset | Name |
|--------|------|
| x.0 | NoSpeedDeviation |
| x.1 | ControlRequested |
| x.2 | SpeedComparisonValusReachedExeeded |
| x.3 | AlarmClassBit0 |
| x.4 | AlarmClassBit1 |
| x.5 | Reserved_Bit13 |
| x.6 | ClosedLoopTorqueControlActive |
| x.7 | Reserved_Bit15 |
| (x+1).0 | ReadyToSwitchOn |
| (x+1).1 | ReadyToOperate |
| (x+1).2 | OperationEnabled |
| (x+1).3 | FaultPresent |
| (x+1).4 | NoCoastStopActivated |
| (x+1).5 | NoQuickStopActivated |
| (x+1).6 | SwitchingOnInhibited |
| (x+1).7 | AlarmPresent |

#### ZSW2 (extended status word)
| Offset | Name |
|--------|------|
| x.0 | TravelToFixedEndStopActive |
| x.1 | DriveDataSetEffectiveBit3 |
| x.2 | DriveDataSetEffectiveBit4 |
| x.3 | MotorDataSetChangeoverActive |
| x.4 | SlaveSignOfLifeBit0 |
| x.5 | SlaveSignOfLifeBit1 |
| x.6 | SlaveSignOfLifeBit2 |
| x.7 | SlaveSignOfLifeBit3 |
| (x+1).0 | DriveDataSetEffectiveBit0 |
| (x+1).1 | DriveDataSetEffectiveBit1 |
| (x+1).2 | DriveDataSetEffectiveBit2 |
| (x+1).3 | Reserved_Bit03 |
| (x+1).4 | RampFunctionGeneratorInactive |
| (x+1).5 | OpenHoldingBrake |
| (x+1).6 | IntegratorInhibit |
| (x+1).7 | ParkingAxisActive |

#### MELDW (message word)
| Offset | Name |
|--------|------|
| x.0 | SpeedTolerance |
| x.1 | Reserved_Bit09 |
| x.2 | Reserved_Bit10 |
| x.3 | ControllerEnable |
| x.4 | DriveReady |
| x.5 | PulsesEnabled |
| x.6 | Reserved_Bit14 |
| x.7 | Reserved_Bit15 |
| (x+1).0 | RampUpDownCompleted |
| (x+1).1 | TorqueUtilizationOk |
| (x+1).2 | ActSpeedValue3Ok |
| (x+1).3 | ActSpeedValue2Ok |
| (x+1).4 | Reserved_Bit04 |
| (x+1).5 | VariableFunctionality |
| (x+1).6 | NoWarningMotorTemperature |
| (x+1).7 | NoPowerUnitOvertemperature |

#### G1_ZSW (encoder 1 status word)
| Offset | Name |
|--------|------|
| x.0 | Probe1Deflected |
| x.1 | Probe2Deflected |
| x.2 | Reserved_Bit10 |
| x.3 | EncoderFaultAcknowledgeActive |
| x.4 | HomePositionExecuted |
| x.5 | AbsoluteValueCyclicallyExecuted |
| x.6 | ParkingSensorExecuted |
| x.7 | SensorError |
| (x+1).0 | Function1Active |
| (x+1).1 | Function2Active |
| (x+1).2 | Function3Active |
| (x+1).3 | Function4Active |
| (x+1).4 | Value1Available |
| (x+1).5 | Value2Available |
| (x+1).6 | Value3Available |
| (x+1).7 | Value4Available |


## DriveSimDesigner_SERVO.fmu variables
Extracted from `modelDescription.xml` inside `simotion FMU\DriveSim_Designer_CODE_v2202\DriveSimDesigner_SERVO.fmu` (FMI 2.0, Co-Simulation only). Value references (VR) are used with `FmuCoSimulation.GetXxx/SetXxx` in [simotion.cs](simotion.cs).

### Inputs
| VR | Name | Type | Unit | Start |
|----|------|------|------|-------|
| 1 | STW1 | Integer | | 0 |
| 2 | STW2 | Integer | | 0 |
| 3 | NSOLL_A | Integer | | 0 |
| 4 | NSOLL_B | Integer | | 0 |
| 5 | G1_STW | Integer | | 0 |
| 6 | G2_STW | Integer | | 0 |
| 7 | MOMRED | Integer | | 0 |
| 8 | XERR | Integer | | 0 |
| 9 | KPC | Integer | | 0 |
| 10 | M_ADD1 | Integer | | 0 |
| 11 | M_LIMIT_POS | Integer | | 0 |
| 12 | M_LIMIT_NEG | Integer | | 0 |
| 13 | S_STW1 | Integer | | 0 |
| 14 | S_STW2 | Integer | | 0 |
| 15 | supply_ready | Boolean | | false |
| 16 | n_act_1 | Real | 1/min | 0 |
| 17 | x_act_1 | Real | | 0 |
| 18 | x_act_2 | Real | | 0 |
| 19 | n_set_ext | Real | 1/min | 0 |
| 20 | M_load_ext | Real | Nm | 0 |
| 21 | par_read_num | Integer | | 0 |
| 22 | par_write_num | Integer | | 0 |
| 23 | par_write_val | Real | | 0 |

### Outputs
| VR | Name | Type | Unit |
|----|------|------|------|
| 24 | ZSW1 | Integer | |
| 25 | ZSW2 | Integer | |
| 26 | NIST_A | Integer | |
| 27 | NIST_B | Integer | |
| 28 | G1_ZSW | Integer | |
| 29 | G1_XIST1 | Integer | |
| 30 | G1_XIST2 | Integer | |
| 31 | G2_ZSW | Integer | |
| 32 | G2_XIST1 | Integer | |
| 33 | G2_XIST2 | Integer | |
| 34 | MELDW | Integer | |
| 35 | M_ACT | Integer | |
| 36 | S_ZSW1 | Integer | |
| 37 | S_ZSW2 | Integer | |
| 38 | n_ctrl_n_set | Real | 1/min |
| 39 | n_set_before_filter | Real | 1/min |
| 40 | n_set_after_filter | Real | 1/min |
| 41 | M_air_gap | Real | Nm |
| 42 | n_act_intMot | Real | 1/min |
| 43 | n_act_intLoad | Real | 1/min |
| 44 | x_act_intMot | Real | 1/min |
| 45 | x_act_intLoad | Real | 1/min |
| 46 | par_read_val | Real | |
| 47 | mismatch | Integer | |

### Parameters
| VR | Name | Unit | Start | Description |
|----|------|------|-------|--------------|
| 48 | Config.ExternalLoad | | 1 | 1 .. external application, 0 .. internal feedback loop |
| 49 | Config.SetpointChannel | | 1 | 1 .. ramp function generator is active, 0 .. ramp function generator is stubbed |
| 50 | Config.SequenceControl | | 1 | 1 .. sequence controller is active, 0 .. sequence controller is stubbed |
| 51 | Config.SpeedController | | 0 | 0 .. speed contoller is stubbed, 1 .. speed contoller is active |
| 52 | Config.CurrentSetpointFilterPar | | 1 | 0 .. parameters for general order filter (Var), 1 .. SINAMICS parameters (Param) |
| 53 | Config.SupplyMode | | 0 | 0 .. not relevant, 1 .. external, 2 .. internal |
| 54 | Config.Tuning | | 0 | 0 .. tuning off, 1 .. tuning enabled, p1460, p1462, p1433, p1434, p1435, CurrentSetPointFilter1 |
| 55 | Param.Motor.p0341_Mot_m_inert | kgm^2 | 0 | Motor moment of inertia |
| 56 | Param.Motor.p0342_Mot_m_inert_ratio | | 1 | Ratio between the total and motor moment of inertia |
| 57 | Param.Commissioning.p0112_t_samp_for_p0115 | | 3 | Sampling times pre-setting p0115 |
| 58 | Param.EncoderEvaluation.p0400_0_Enc_type_sel | | 0 | Encoder 1 type selection |
| 59 | Param.EncoderEvaluation.p0400_1_Enc_type_sel | | 0 | Encoder 2 type selection |
| 60 | Param.EncoderEvaluation.p0418_0_Enc_fine_Gx_XIST1 | | 11 | Fine resolution G1_XIST (in bits) |
| 61 | Param.EncoderEvaluation.p0418_1_Enc_fine_Gx_XIST1 | | 11 | Fine resolution G2_XIST (in bits) |
| 62 | Param.EncoderEvaluation.p0419_0_Enc_fine_Gx_XIST2 | | 9 | Fine resolution absolute value Gx_XIST2 (in bits) |
| 63 | Param.EncoderEvaluation.p0419_1_Enc_fine_Gx_XIST2 | | 9 | Fine resolution absolute value Gx_XIST2 (in bits) |
| 64 | Param.PROFIdrive.p0922_IF1_PZD_tel | | 1 | IF1 PROFIdrive PZD telegram selection |
| 65 | Param.PROFIdrive.p8864_IF1_Pd_1_sup_tel | | 0 | IF1 PROFIdrive first supplementary telegram selection |
| 66 | Param.SetpointChannel.p1082_n_max | 1/min | 2000 | Maximum speed |
| 67 | Param.SetpointChannel.p1120_RFG_ramp_up_time | s | 10 | Ramp-function generator ramp-up time |
| 68 | Param.SetpointChannel.p1121_RFG_ramp_down_time | s | 10 | Ramp-function generator ramp-down time |
| 69 | Param.SetpointChannel.p1135_OFF3_t_RD | s | 0 | OFF3 ramp-down time |
| 70 | Param.SetpointChannel.p1148_RFG_tol_HL_RL_act | 1/min | 19.8 | Ramp-function gen. tolerance for ramp-up and ramp-down active |
| 71 | Param.Functions.p1215_Brake_config | | 0 | Motor holding brake configuration |
| 72 | Param.Functions.p1226_n_standst_n_thr | 1/min | 20 | Standstill detection speed threshold |
| 73 | Param.Functions.p1227_n_standst_t_mon | s | 4 | Zero speed detection monitoring time |
| 74 | Param.Functions.p1228_Pulse_suppr_t_del | s | 0 | Pulse suppression delay time |
| 75 | Param.ClosedLoopControl.p1402_I_ctrl_config | | 0 | Bit 4: 16 - Closed-loop current control and motor model configuration |
| 76 | Param.ClosedLoopControl.p1433_n_ctrl_RefMod_fn | Hz | 0 | Speed controller reference model natural frequency |
| 77 | Param.ClosedLoopControl.p1434_n_ctrl_RefMod_D | | 1 | Speed controller reference model damping |
| 78 | Param.ClosedLoopControl.p1435_n_ctrRefMod_t_dead | | 0 | Speed controller reference model dead time |
| 79 | Param.ClosedLoopControl.p1460_n_ctrl_Kp_n_lower | Nms/rad | 0.3 | Speed controller P gain adaptation speed lower |
| 80 | Param.ClosedLoopControl.p1462_n_ctrl_Tn_n_lower | ms | 20 | Speed contr. integral act. time adaptation speed lower |
| 81 | Param.ClosedLoopControl.p1497_CI_M_inert_scal_s_s | % | 100 | Moment of inertia scaling signal source |
| 82 | Param.ClosedLoopControl.p1498_Load_m_inert | kgm^2 | 0 | Load moment of inertia |
| 83 | Param.ClosedLoopControl.p1520_M_max_upper_mot | Nm | 14.5 | Torque limit upper |
| 84 | Param.ClosedLoopControl.p1521_M_max_lower_reg | Nm | -14.5 | Torque limit lower |
| 85 | Param.ClosedLoopControl.p1530_P_max_mot | kW | 3 | Power limit motoring |
| 86 | Param.ClosedLoopControl.p1531_P_max_reg | kW | -3 | Power limit regenerative |
| 87 | Param.ClosedLoopControl_Filter.p1656_I_set_filt_act | | 0 | Activates current setpoint filter |
| 88 | Param.ClosedLoopControl_Filter.p1658_I_set_filt_1_fn_den | Hz | 1999 | Current setpoint filter 1 denominator natural frequency |
| 89 | Param.ClosedLoopControl_Filter.p1659_I_set_filt_1_D_den | | 0.7 | Current setpoint filter 1 denominator damping |
| 90 | Param.ClosedLoopControl_Filter.p1660_I_set_filt_1_fn_num | Hz | 1999 | Current setpoint filter 1 numerator natural frequency |
| 91 | Param.ClosedLoopControl_Filter.p1661_I_set_filt_1_D_num | | 0.7 | Current setpoint filter 1 numerator damping |
| 92 | Param.ClosedLoopControl_Filter.p1663_I_set_filt_2_fn_den | Hz | 1999 | Current setpoint filter 2 denominator natural frequency |
| 93 | Param.ClosedLoopControl_Filter.p1664_I_set_filt_2_D_den | | 0.7 | Current setpoint filter 2 denominator damping |
| 94 | Param.ClosedLoopControl_Filter.p1665_I_set_filt_2_fn_num | Hz | 1999 | Current setpoint filter 2 numerator natural frequency |
| 95 | Param.ClosedLoopControl_Filter.p1666_I_set_filt_2_D_num | | 0.7 | Current setpoint filter 2 numerator damping |
| 96 | Param.ReferenceValues.p2000_n_ref | 1/min | 2000 | Reference speed |
| 97 | Param.ReferenceValues.p2003_M_ref | Nm | 14.5 | Reference torque |
| 98 | Param.Communication.p2048_IF1_PZD_t_samp | ms | 1 | IF1 PROFIdrive PZD sampling time |
| 99 | Param.SafetyIntegrated.p9852_SI_Stop_1_t_del_MM | ms | 0 | SI Safe Stop 1 delay time (Motor Module) |
| 100 | Param.SafetyIntegrated.p60022_Ps_tel_sel | | 0 | PROFIsafe telegram selection |
| 101 | Var.InternalLoad.InertiaMotor | kgm^2 | 0.005 | Inertia of motor and brake |
| 102 | Var.InternalLoad.InertiaLoad | kgm^2 | 0.008 | Inertia of the load |
| 103 | Var.InternalLoad.InertiaRatio | | 1 | Inertia ratio to define InertiaMotor (alternative) |
| 104 | Var.InternalLoad.GearBoxRatio | | 1 | Gear box ratio |
| 105 | Var.InternalLoad.Damping | | 0.03 | Damping (modal) |
| 106 | Var.InternalLoad.TorsionalStiffness | Nm/rad | 2000 | Torsional Stiffness |
| 107 | Var.InternalLoad.AbsorberFrequency | Hz | 0 | Tilger frequency of two mass oszilator [Hz] |
| 108 | Var.InternalLoad.BaseLoad_linear | Nm | 0 | Base load [Nm] for lineare characteristic |
| 109 | Var.InternalLoad.BaseLoad_square | Nm | 0 | Base load [Nm] for square characteristic |
| 110 | Var.InternalLoad.maxLoadSpeed | 1/min | 2000 | Max. load speed [1/min] |
| 111 | Var.InternalLoad.negFixedStop | | 0 | negative postion in revolutions for fixed stop |
| 112 | Var.InternalLoad.posFixedStop | | 0 | positive postion in revolutions for fixed stop |
| 113 | Var.InternalLoad.SampleTime_M_Load_ext | ms | 1 | FMU co-simulation time [ms] |
| 114 | Var.CurrentSetPointFilter1.Active | | 0 | 0 .. Filter 1 inactive, 1 .. Filter 1 inactive |
| 115 | Var.CurrentSetPointFilter1.BlockingFrequency | Hz | 500 | Blocking frequency f_sp [Hz] of filter 1 |
| 116 | Var.CurrentSetPointFilter1.Bandwidth | Hz | 500 | Bandwidth f_bb [Hz] of filter 1 |
| 117 | Var.CurrentSetPointFilter1.NotchDepth | | -20 | Notch depth K [dB] of filter 1 |
| 118 | Var.CurrentSetPointFilter1.ReductionAbs | | -20 | Reduction Abs [dB] of filter 1 |
| 119 | Var.CurrentSetPointFilter2.Active | | 0 | 0 .. Filter 2 inactive, 1 .. Filter 2 inactive |
| 120 | Var.CurrentSetPointFilter2.BlockingFrequency | Hz | 500 | Blocking frequency f_sp [Hz] of filter 2 |
| 121 | Var.CurrentSetPointFilter2.Bandwidth | Hz | 500 | Bandwidth f_bb [Hz] of filter 2 |
| 122 | Var.CurrentSetPointFilter2.NotchDepth | | -20 | Notch depth K [dB] of filter 2 |
| 123 | Var.CurrentSetPointFilter2.ReductionAbs | | -20 | Reduction Abs [dB] of filter 2 |
| 124 | Var.InternalSupply.PreChargeDuration | s | 0.05 | Estimated precharge duration [s] |

