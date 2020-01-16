package frc.robot;

import frc.lib.motorcontroller.WrappedSparkMax;
import frc.lib.utils.Utilities;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Class to contain all functions that affect the drivetrain.
 */
public class Drivetrain {

    public static Drivetrain instance;
    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }
    
    private Drivetrain() {

    }

    private RobotMap map = RobotMap.getInstance();

    private WrappedSparkMax rightSide = map.rightMaster;
    private WrappedSparkMax leftSide = map.leftMaster;

    private DrivetrainControlState controlState = DrivetrainControlState.PERCENT_OUTPUT;

    private boolean isCurvatureDrive = true;

    private double kSteeringDeadband = 0.2;
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

    private double rightOutput, leftOutput;

    public void updateOutputs(double throttle, double steering, boolean isQuickTurn) {
        if (isCurvatureDrive) {
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

            leftOutput = leftPWR;
            rightOutput = rightPWR;
        } else {
            leftOutput = throttle + steering;
            rightOutput = throttle - steering;
        }
    }

    public void setOutputs() {
        rightSide.setReference(rightOutput);
        leftSide.setReference(leftOutput);
    }

    public enum DrivetrainControlState {
        PERCENT_OUTPUT,
        VOLTAGE_CONTROL,
        VELOCITY_CONTROL,
        POSITION_CONTROL,
        MOTION_MAGIC;
    }
    
}