package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotMap;
import frc.robot.Constants;

public class Hood implements Updateable {
    
    public static Hood kInstance;

    public static Hood getInstance() {
        if (kInstance == null) {
            kInstance = new Hood();
        }
        return kInstance;
    }

    private RobotMap map;

    private TalonSRX hood;
    private double outputValue = 0.0;;

    public DigitalInput hoodLimit;

    private HoodState state = HoodState.Zeroing;
    private HoodControlState controlState = HoodControlState.PercentOutput;

    private Hood() {
        map = RobotMap.getInstance();

        hood = map.hood;
        hoodLimit = map.hoodLimit;

        hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
        hood.setInverted(true);
        hood.setSensorPhase(false);
        hood.configNominalOutputForward(0);
        hood.configPeakOutputForward(100);
        //Flywheel should never move in revearse
        hood.configNominalOutputReverse(0);
        hood.configPeakOutputReverse(0);

        hood.config_kF(0, Constants.kHoodF, Constants.kTimeoutMs);
        hood.config_kP(0, Constants.kHoodP, Constants.kTimeoutMs);
        hood.config_kI(0, Constants.kHoodI, Constants.kTimeoutMs);
        hood.config_kD(0, Constants.kHoodD, Constants.kTimeoutMs); 
    }
    
    public void update(double dt) {

    }

    public void dashboard() {
        
    }

    public void setState(HoodState newState) {

    }

    public void setControlState(HoodControlState newState) {
        
    }

    public static enum HoodState {
        Raising,
        Lowering,
        Holding,
        Automatic,
        Zeroing;
    }

    public static enum HoodControlState {
        PercentOutput,
        VelocityPID,
        PositionPID,
        MotionMagic;
    }

}