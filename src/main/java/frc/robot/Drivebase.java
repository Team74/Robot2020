package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivebase implements Updateable {
    private static Drivebase kInstance = null;
    private InputManager inputManager;
    private RobotMap robotMap;
    private DifferentialDrive differentialDrive;
    
    private SpeedControllerGroup leftMotor, rightMotor;

    private double driveScaler = 1;
    private double turnScaler = driveScaler;

    private DriveState driveState = DriveState.Teleop;
    
    public static Drivebase getInstance() {
        if (kInstance == null) {
            kInstance = new Drivebase();
        } 
        return kInstance;
    }

    public Drivebase() {
        inputManager = Robot.inputManager;
        robotMap = RobotMap.getInstance();
        leftMotor = robotMap.driveLeft; 
        rightMotor = robotMap.driveRight;

        differentialDrive = new DifferentialDrive(leftMotor, rightMotor);
    }

    public void dashboard() {
        
    }

    public void update(double dt) {
        // differentialDrive.arcadeDrive(-driveScaler * inputManager.driverRightStickX, turnScaler * inputManager.driverLeftStickY);
        differentialDrive.tankDrive(driveScaler * inputManager.driverLeftStickY, driveScaler * inputManager.driverLeftStickY);

        if (inputManager.driverLeftBumper) {
            handleShift(ShiftState.Low);
          } else if (inputManager.driverRightBumper) {
            handleShift(ShiftState.High);
          }
     
          if (inputManager.driverTriggerRight > .85) {
           driveScaler = .5;
          } else {
           driveScaler = 1;
          }
          turnScaler = driveScaler;
    }

    public void driveDistance(double inches) {

    }

    public void turnToAngle(double angleDegrees) {

    }
    
    public void handleShift(ShiftState newState) {
        switch(newState) {
            case High:
                // robotMap.gearShift.set(Value.kForward);
                break;
            case Low:
                // robotMap.gearShift.set(Value.kReverse);
                break;
            default:
                // robotMap.gearShift.set(Value.kReverse);
                break;
        }
    }

    public void setDriveState(DriveState newState) {
        if (driveState == newState) {
            System.out.println("Drive State already equals " + newState);
        }
        driveState = newState;
    }

    public enum ShiftState {
        High,
        Low;
    }

    public enum DriveState {
        PathFollowing,
        Autonomous,
        Teleop;
    }
}