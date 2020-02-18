package frc.lib.motorcontroller;

import frc.lib.motorcontroller.TalonOverride;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


//Add suport for victorspx
public class TalonSRXBuilder {

    private static int kTimeoutMs = 30;

    //Defualt configs to use
    //I'll add more when I think of them
    //Test before implementing
    private static NeutralMode neutralMode = NeutralMode.Brake;

    private static final double openLoopRampRate = 0.0;//In seconds from 0 to full power
    private static final double closedLoopRampRate = 0.0;//In seconds fron 0 to full power

    private static final boolean enableCurrentLimit = false;
    private static final boolean limitSwitchesEnable = false;
    private static final boolean enableSoftLimits = false;
    private static final boolean invertMotor = false;
    private static final boolean invertSensor = false;
    //private static

    public static TalonSRX buildDefaultTalon(int canID) {
        return makeTalon(canID);
    }

    public static TalonSRX buildSlavedTalon(int canID, int masterID) {
        TalonSRX talon = makeTalon(canID);
        talon.set(ControlMode.Follower, masterID);
        return talon;
    }

    private static TalonSRX makeTalon(int canID){
        TalonSRX talon = new TalonOverride(canID);
        talon.setNeutralMode(neutralMode);
        talon.configOpenloopRamp(openLoopRampRate, kTimeoutMs);;
        talon.configClosedloopRamp(closedLoopRampRate, kTimeoutMs);;

        //Set max and min outputs for each direction
        talon.configNominalOutputForward(0, kTimeoutMs);;
        talon.configPeakOutputForward(1.0, kTimeoutMs);;

        talon.configNominalOutputReverse(0, kTimeoutMs);;
        talon.configPeakOutputReverse(-1.0, kTimeoutMs);;

        talon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, kTimeoutMs);;
        talon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, kTimeoutMs);;
        talon.enableCurrentLimit(enableCurrentLimit);
        talon.overrideLimitSwitchesEnable(limitSwitchesEnable);
        talon.overrideSoftLimitsEnable(enableSoftLimits);
        talon.configNeutralDeadband(0.04);//0.04 is factory default

        //Initally disable these, can be modified later. This just prevents unwanted behavior by accident
        talon.setInverted(invertMotor);
        talon.setSensorPhase(invertSensor);

        talon.configVoltageCompSaturation(0.0, kTimeoutMs);
        talon.enableVoltageCompensation(false);

        talon.clearMotionProfileHasUnderrun(kTimeoutMs);
        talon.clearMotionProfileTrajectories();

        talon.clearStickyFaults(kTimeoutMs);

        return talon;
    }
    
}