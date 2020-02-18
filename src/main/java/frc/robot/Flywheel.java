package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotMap;
import frc.lib.motorcontroller.TalonSRXBuilder;
import frc.robot.Constants;

public class Flywheel implements Updateable {
    
    public static Flywheel kInstance;

    public static Flywheel getInstance() {
        if (kInstance == null) {
            kInstance = new Flywheel();
        }
        return kInstance;
    }

    private RobotMap map;

    private TalonSRX flywheel;
    private double outputValue = 0.0;

    private FlywheelState state;
    private FlywheelControlState controlState;

    private Flywheel() {
        map = RobotMap.getInstance();
        flywheel = map.flywheel;

        flywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
        flywheel.setInverted(false);
        flywheel.setSensorPhase(true);
        flywheel.configNominalOutputForward(0);
        flywheel.configNominalOutputReverse(0);
        flywheel.configPeakOutputForward(100);
        flywheel.configPeakOutputReverse(0);

        flywheel.config_kF(0, Constants.kFlywheelF, Constants.kTimeoutMs);
        flywheel.config_kP(0, Constants.kFlywheelP, Constants.kTimeoutMs);
        flywheel.config_kI(0, Constants.kFlywheelI, Constants.kTimeoutMs);
        flywheel.config_kD(0, Constants.kFlywheelD, Constants.kTimeoutMs);    
    }

    public void update(double dt) {
        switch(controlState) {
            case PercentOutput:
                flywheel.set(ControlMode.PercentOutput, outputValue);
                break;
            case VelocityPID:
                flywheel.set(ControlMode.Velocity, Math.round(outputValue));
                break;
            case PositionPID:
                flywheel.set(ControlMode.Position, Math.round(outputValue));
                break;
            case MotionMagic:
                flywheel.set(ControlMode.MotionMagic, Math.round(outputValue));
                break;
            default:
                flywheel.stopMotor();
                break;
        }
    }

    public void dashboard() {
        
    }

    /**
     * Sets the motor controller, param units vary depending on the value of controlState
     * @param outputValue Value to pass to the motor controller
     */
    public void set(double outputValue) {

    }

    public void setState(FlywheelState newState) {
        if (state == newState) {
            System.out.println("Flywheel state already equals " + newState);
        } else {
            state = newState;
        }
    }

    public void setControlState(FlywheelControlState newState) {
        if (controlState == newState) {
            System.out.println("Flywheel control state already equals " + newState);
        } else {
            controlState = newState;
        }
    }

    public static enum FlywheelState {

    }

    public static enum FlywheelControlState {
        PercentOutput,
        VelocityPID,
        PositionPID,
        MotionMagic;
    }

}