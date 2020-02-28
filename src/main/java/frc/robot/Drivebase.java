package frc.robot;

import java.util.HashMap;

import com.revrobotics.*;
import com.kauailabs.navx.frc.AHRS;

public class Drivebase implements Updateable {
    private static Drivebase kInstance = null;
    private InputManager inputManager;
    private RobotMap robotMap;
    
    private CANSparkMax leftMaster, leftFollower, rightMaster, rightFollower;
    public CANEncoder leftEncoder, rightEncoder;
    public CANPIDController leftController, rightController;

    private AHRS gyro;
  
    private double driveScalar = 1;
    private double turnScalar = .75;

    private DriveState driveState = DriveState.Teleop;

    private double autoTurnScalar = 0.5;
    private double autoTurnDeadband = 0.5;

    //The value needed to convert from distance, to neo encoder ticks
    private double distanceConversionFactor = 1.0;
    private double autoDistanceDeadband = 0.5;

    private boolean atTarget = false;

    private HashMap<String, Integer> drivePIDSlots = new HashMap<>() {
        private static final long serialVersionUID = -9999;
        {
            put("SmartMotion", 0);
            put("Veloicty", 1);
            put("Position", 2);
            put("Voltage", 3);
        }
    };
    
    public static Drivebase getInstance() {
        if (kInstance == null) {
            kInstance = new Drivebase();
        } 
        return kInstance;
    }

    public Drivebase() {
        inputManager = Robot.inputManager;
        robotMap = RobotMap.getInstance();

        leftMaster = robotMap.leftDriveMaster;
        leftFollower = robotMap.leftDriveFollower ;
        rightMaster = robotMap.rightDriveMaster;
        rightFollower = robotMap.rightDriveFollower;

        leftMaster.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightMaster.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();

        leftController = leftMaster.getPIDController();
        rightController = leftMaster.getPIDController();

        leftMaster.setInverted(true);
        rightMaster.setInverted(false);

        leftFollower.follow(leftMaster, false);
        rightFollower.follow(rightMaster, false);

        gyro = robotMap.navX;
    }

    public void dashboard() {
        
    }

    public void update(double dt) {

        switch(driveState) {
            case Teleop:
                if (inputManager.driverLeftBumper) {
                    handleShift(ShiftState.Low);
                } else if (inputManager.driverRightBumper) {
                    handleShift(ShiftState.High);
                }
                arcadeDrive(-inputManager.driverRightStickX, inputManager.driverRightStickY, true, (inputManager.driverTriggerRight > .85));
                leftMaster.set(DriveCommands.leftMotorOutput);
                rightMaster.set(DriveCommands.rightMotorOutput);
                break;
            case DriveStraight:
                leftController.setReference(DriveCommands.targetDistance, ControlType.kSmartMotion, drivePIDSlots.get("SmartMotion"));
                rightController.setReference(DriveCommands.targetDistance, ControlType.kSmartMotion, drivePIDSlots.get("SmartMotion"));
                this.setAtTarget(Math.abs(((leftEncoder.getPosition() + rightEncoder.getPosition()) / 2) - DriveCommands.targetDistance) <= autoDistanceDeadband);
                break;
            case TurnAngle:
                double heading = this.getHeading();
                double headingDiff = heading + DriveCommands.targetHeading;
                //CLamp output to the [-1.0, 1.0] range and apply a porpotionality constant
                double output = Math.max(-1.0, Math.min(1.0, (autoTurnScalar * headingDiff)));
                leftMaster.set(output);
                rightMaster.set(-output);
                this.setAtTarget(Math.abs(headingDiff) <= autoTurnDeadband);
                break;
            default:
                break;
        }
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public void zeroDriveEncoders() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }

    /**
     * Method to get the gyro angle and bound it 0 to 360
     * @return Bounded gyro angle
     */
    public double getHeading() {
        return gyro.getAngle() % 360;
    }

    public void setAtTarget(boolean value) {
        atTarget = value;
    }

    public boolean atTarget() {
        return this.atTarget;
    }

    public void setTargetDistance(double inches) {
        DriveCommands.targetDistance = inches;
    }

    public void setTargetHeading(double angleDegrees) {
        DriveCommands.targetHeading = angleDegrees;
    }

    public void arcadeDrive(double throttle, double wheel, boolean squareInputs, boolean scaleInputs) {
        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
            throttle = Math.copySign(throttle * throttle, throttle);
            wheel = Math.copySign(wheel * wheel, wheel);
        }
        double tempDriveScalar = driveScalar;
        double tempTurnScalar = turnScalar;
        if (scaleInputs) {
            tempDriveScalar = driveScalar * 0.5;
            tempTurnScalar = turnScalar * 0.5;
        }
        throttle *= tempDriveScalar;
        wheel *= tempTurnScalar; 
            
        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(wheel)), throttle);

        if (throttle >= 0.0) {
            // First quadrant, else second quadrant
            if (wheel >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = throttle - wheel;
            } else {
                leftMotorOutput = throttle + wheel;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (wheel >= 0.0) {
                leftMotorOutput = throttle + wheel;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = throttle - wheel;
            }
        }

        DriveCommands.leftMotorOutput = leftMotorOutput;
        DriveCommands.rightMotorOutput = rightMotorOutput;
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

    public static class DriveCommands {
        public static double leftMotorOutput;
        public static double rightMotorOutput;

        public static double targetHeading; //Degrees
        public static double targetDistance; //Inches 
    }

    public enum ShiftState {
        High,
        Low;
    }

    public enum DriveState {
        PathFollowing,
        DriveStraight,
        TurnAngle,
        Teleop;
    }
}