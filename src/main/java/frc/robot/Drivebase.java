package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivebase implements Updateable {
    private InputManager inputManager;
    private RobotMap robotMap;
    private DifferentialDrive differentialDrive;
    
    private SpeedControllerGroup leftMotor, rightMotor;

    private double driveScaler = 1;
    private double turnScaler = driveScaler * .8;

    public Drivebase(InputManager im) {
        inputManager = im;
        robotMap = RobotMap.getInstance();
        leftMotor = robotMap.driveLeft; 
        rightMotor = robotMap.driveRight;

        differentialDrive = new DifferentialDrive(leftMotor, rightMotor);
    }

    public void update(double dt) {
        differentialDrive.arcadeDrive(inputManager.driverLeftStickY, -inputManager.driverRightStickX);

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
          turnScaler = driveScaler * .8;
    }
    
    public void handleShift(ShiftState newState) {
        switch(newState) {
            case High:
                robotMap.gearShift.set(Value.kForward);
                break;
            case Low:
                robotMap.gearShift.set(Value.kReverse);
                break;
            default:
                robotMap.gearShift.set(Value.kReverse);
                break;
        }
    }

    private enum ShiftState {
        High,
        Low;
    }
}