package frc.robot;

import java.util.HashMap;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
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

    private double autoTurnScalar = 1.0/90.0;
    private double autoTurnDeadband = 0.5;

    private double kThrottleDeadband = 0.2;
    private double kSteeringNonlinearity = 0.5;

    private final double kNegInertiaThreshold = 0.65;
    private final double kNegInertiaTurnScalar = 3.5;
    private final double kNegInertiaCloseScalar = 4.0;
    private final double kNegInertiaFarScalar = 5.0;

    private final double kSensitivity = 0.65;

    private final double kQuickStopDeadband = 0.5;
    private final double kQuickStopWeight = 0.1;
    private final double kQuickStopScalar = 5.0;

    private double oldSteering = 0.0;
    private double quickStopAccumlator = 0.0;
    private double negInertiaAccumlator = 0.0;

    //The value needed to convert from distance, to neo encoder ticks
    private double distanceConversionFactor = 1.0;
    private double autoDistanceDeadband = 50;

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
        inputManager = InputManager.getInstance();
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
        rightController = rightMaster.getPIDController();

        leftMaster.setInverted(true);
        rightMaster.setInverted(false);

        leftMaster.setIdleMode(IdleMode.kCoast);
        leftFollower.setIdleMode(IdleMode.kCoast);
        rightMaster.setIdleMode(IdleMode.kCoast);
        rightFollower.setIdleMode(IdleMode.kCoast);

        leftController.setSmartMotionMaxAccel(500, drivePIDSlots.get("SmartMotion"));
        leftController.setSmartMotionMaxVelocity(5000, drivePIDSlots.get("SmartMotion"));
        leftController.setSmartMotionMinOutputVelocity(0, drivePIDSlots.get("SmartMotion"));

        rightController.setSmartMotionMaxAccel(500, drivePIDSlots.get("SmartMotion"));
        rightController.setSmartMotionMaxVelocity(5000, drivePIDSlots.get("SmartMotion"));
        rightController.setSmartMotionMinOutputVelocity(0, drivePIDSlots.get("SmartMotion"));

        leftController.setFF(1.0/5000.0);
        leftController.setP(0.0);
        leftController.setI(0.0);
        leftController.setD(0.0);

        rightController.setFF(1.0/5000.0);
        rightController.setP(0.0);
        rightController.setI(0.0);
        rightController.setD(0.0);

        leftFollower.follow(leftMaster, false);
        rightFollower.follow(rightMaster, false);

        gyro = robotMap.navX;
    }

    /**
     * Look into refatoring to seperate out inputs into here at a later date.
     */
    public void handleInput() {

    }

    public void dashboard() {
        
    }

    public void update(double dt) {
        // printDriebaseData();
        switch(driveState) {
            case Teleop:
                if (inputManager.driverLeftBumper) {
                    handleShift(ShiftState.Low);
                } else if (inputManager.driverRightBumper) {
                    handleShift(ShiftState.High);
                }
                arcadeDrive(inputManager.driverLeftStickY, inputManager.driverRightStickX, true, (inputManager.driverTriggerRight > .85));
                leftController.setReference(DriveCommands.leftMotorOutput, ControlType.kDutyCycle);
                rightController.setReference(DriveCommands.rightMotorOutput, ControlType.kDutyCycle);
                break;
            case DriveStraight:
                leftController.setReference(DriveCommands.targetDistance, ControlType.kSmartMotion, drivePIDSlots.get("SmartMotion"));
                rightController.setReference(DriveCommands.targetDistance, ControlType.kSmartMotion, drivePIDSlots.get("SmartMotion"));
                //Subtract right because it's encoder is counting negavtive when we go foward when we are in position mode
                this.setAtTarget(Math.abs(((leftEncoder.getPosition() + rightEncoder.getPosition()) / 2) - DriveCommands.targetDistance) <= autoDistanceDeadband);
                break;
            case TurnAngle:
                double heading = this.getHeading();
                double headingDiff = heading - DriveCommands.targetHeading;
                //CLamp output to the [-1.0, 1.0] range and apply a porpotionality constant
                double output = Math.max(-1.0, Math.min(1.0, (autoTurnScalar * headingDiff)));
                leftController.setReference(output, ControlType.kDutyCycle);
                rightController.setReference(-output, ControlType.kDutyCycle);
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
        return gyro.getAngle();// % 360;
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
            throttle = Math.copySign(throttle * throttle *throttle, throttle);
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

        // double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(wheel)), throttle);

        // if (throttle >= 0.0) {
        //     // First quadrant, else second quadrant
        //     if (wheel >= 0.0) {
        //         leftMotorOutput = maxInput;
        //         rightMotorOutput = throttle - wheel;
        //     } else {
        //         leftMotorOutput = throttle + wheel;
        //         rightMotorOutput = maxInput;
        //     }
        // } else {
        //     // Third quadrant, else fourth quadrant
        //     if (wheel >= 0.0) {
        //         leftMotorOutput = throttle + wheel;
        //         rightMotorOutput = maxInput;
        //     } else {
        //         leftMotorOutput = maxInput;
        //         rightMotorOutput = throttle - wheel;
        //     }
        // }

        leftMotorOutput = throttle - wheel;
        rightMotorOutput = throttle + wheel;
        
        // double maxMotorOutput = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        // double normalizedLeftMotorOutput = Math.copySign(leftMotorOutput / maxMotorOutput, leftMotorOutput);
        // double normalizedRightMotorOutput = Math.copySign(leftMotorOutput / maxMotorOutput, leftMotorOutput);

        DriveCommands.leftMotorOutput = leftMotorOutput;
        DriveCommands.rightMotorOutput = rightMotorOutput;
    }

    public void curvatureDrive(double throttle, double steering, boolean isQuickTurn) {
        throttle = Utilities.handleDeadband1D(throttle, kThrottleDeadband);
        steering = Utilities.handleDeadband1D(steering, kSteeringDeadband);
        double negInertia = steering - oldSteering;
        oldSteering = steering;

        final double denominator = Math.sin(Math.PI / 2.0 *kSteeringNonlinearity);
        steering = Math.sin(Math.PI / 2.0 * kSteeringNonlinearity * steering) / denominator;
        steering = Math.sin(Math.PI / 2.0 * kSteeringNonlinearity * steering) / denominator;

        double linearPWR, angularPWR;
        double rightPWR, leftPWR, overPWR;

        double negInertiaScalar;
        if ((steering * negInertia) > 0) {
            negInertiaScalar = kNegInertiaTurnScalar;
        } else {
            if (Math.abs(steering) > kNegInertiaThreshold) {
                negInertiaScalar = kNegInertiaFarScalar;
            } else {
                negInertiaScalar = kNegInertiaCloseScalar;
            }
        }
        double negInertiaPWR = negInertia * negInertiaScalar;
        negInertiaAccumlator += negInertiaPWR;

        steering = steering + negInertiaAccumlator;
        if (negInertiaAccumlator > 1) {
            negInertiaAccumlator -= 1;
        } else if (negInertiaAccumlator < -1) {
            negInertiaAccumlator += 1;
        } else {
            negInertiaAccumlator = 0;
        }
        linearPWR = throttle;


        if (isQuickTurn) {
            if(Math.abs(linearPWR) < kQuickStopDeadband) {
                quickStopAccumlator = (1 - kQuickStopWeight) * quickStopAccumlator + kQuickStopWeight * Utilities.limit(steering, 1.0) * kQuickStopScalar;
            }
            overPWR = 1.0;
            angularPWR = steering;
        } else {
            overPWR = 0.0;
            angularPWR = Math.abs(linearPWR) * steering * kSensitivity - quickStopAccumlator;
            if (quickStopAccumlator > 1) {
                quickStopAccumlator -= 1;
            } else if (quickStopAccumlator < -1) {
                quickStopAccumlator += 1;
            } else {
                quickStopAccumlator = 0.0;
            }
        }
        rightPWR = leftPWR = linearPWR;
        leftPWR += angularPWR;
        rightPWR -= angularPWR;

        if (leftPWR > 1.0) {
            rightPWR -= overPWR * (leftPWR - 1.0);
            leftPWR = 1.0;
        } else if (rightPWR > 1.0) {
            leftPWR -= overPWR * (rightPWR - 1.0);
            rightPWR = 1.0;
        } else if (leftPWR < -1.0) {
            rightPWR += overPWR * (-1.0 - leftPWR);
            leftPWR = -1.0;
        } else if (rightPWR < -1.0) {
            leftPWR += overPWR * (-1.0 - rightPWR);
            rightPWR = -1.0;
        }

        DriveCommands.leftMotorOutput = leftPWR;
        DriveCommands.rightMotorOutput = rightPWR;
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

    public void printDriebaseData() {
        System.out.println("Left Position: " + leftEncoder.getPosition());
        System.out.println("Right Position: " + rightEncoder.getPosition());
        System.out.println("Left Velocity: " + leftEncoder.getVelocity());
        System.out.println("Right Velocity: " + rightEncoder.getVelocity());
        // System.out.println("Heading: " + getHeading());
    }

    public static class DriveCommands {
        public static double leftMotorOutput;
        public static double rightMotorOutput;

        public static double targetHeading; //Degrees
        public static double targetDistance; //Inches 
    }

    public static enum ShiftState {
        High,
        Low;
    }

    public static enum DriveState {
        PathFollowing,
        DriveStraight,
        TurnAngle,
        Teleop;
    }
}

private double kSteeringDeadband = 0.2;


    public void updateOutputs(double throttle, double steering, boolean isQuickTurn) {
        if (isCurvatureDrive) {
            