package frc.robot;

import java.util.HashMap;

import javax.management.RuntimeErrorException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Superstructure implements Updateable {
    private static Superstructure kInstance = null;

    private BaseMotorController flywheel, hood, intake, indexer, uptake;
    private TalonSRX turret;

    private DigitalInput hoodLimit, indexerRotationLimit;
    private DigitalInput[] ballLimits;

    private InputManager inputManager;
    private Vision vision;

    private HashMap<String, Double> highPort = new HashMap<>() {
        private static final long serialVersionUID = -9999;
        {
            put("camera_angle", 1.0);
            put("camera_height", 1.0);
            put("target_height", 89.75);
        }
    };
    private Boolean flywheelOn = false;
    private IntakeState intakeState = IntakeState.IntakeUp;
    private int turretState = 0;
    private HoodState hoodState = HoodState.Zeroing;
    private HoodControlState hoodControlState = HoodControlState.PercentOutput;
    private boolean isAdvancing = false;
    private ShooterState shootState = ShooterState.NotShooting;
    private ShooterControlState shooterControlState = ShooterControlState.VelocityPID;
    private Boolean shooterOn = false;
    private Boolean shooterAlign = false;
    private IndexerState indexerState = IndexerState.NoBalls;

    private boolean intakeFrwd = false;
    private boolean intakeRev = false;

    private int flywheelHold = 0;
    private boolean driverAhold = false;
    private int indexerHold = 0;

    private int maximumRotation = 891324;
    private boolean hasHitLeft = false;
    private double targetDeadband = 0.5;

    private TurretState autoTurretState = TurretState.Holding;

    private NetworkTable limelight;
    private NetworkTableEntry tx, ty, ta, tv;

    //-27 to 27 degrees
    private double targetAngleHorizontal;
    //-20.5 to 20.5 degrees
    private double targetAngleVertical;
    private double targetAreaScalar;
    //0 no valid targets, 1 valid target
    private double validTargets;

    public Superstructure() {
        flywheel = Robot.robotMap.flywheel;
        turret = Robot.robotMap.turret;
        hood = Robot.robotMap.hood;
        intake = Robot.robotMap.intake;
        indexer = Robot.robotMap.indexer;
        uptake = Robot.robotMap.uptake;
        
        hoodLimit = Robot.robotMap.hoodLimit;
        indexerRotationLimit = Robot.robotMap.indexerRotationLimit;
        ballLimits = Robot.robotMap.ballLimit;

        inputManager = Robot.inputManager;

        limelight = Robot.robotMap.limelight;

        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
        tv = limelight.getEntry("tv");
    }

    public static Superstructure getInstance() {
        if (kInstance == null) {
            kInstance = new Superstructure();
        } 
        return kInstance;
    }

    public void dashboard() {
 
    }

    public void handleInput() {
        //Flywheel
        if (inputManager.opX) {
            if (flywheelOn && flywheelHold == 0) {
              flywheelOn = false;
            } else if (flywheelHold == 0) {
              flywheelOn = true;
            }
            flywheelHold++;
        } else {
            flywheelHold = 0;
        }

        //Testing zeroing hood
        if (inputManager.driverX) {
            hoodState = HoodState.Zeroing;
        }

        //Intake
        if (inputManager.driverA && !intakeFrwd && !driverAhold) {
            intakeFrwd = true;
            driverAhold = true;
        } else if (inputManager.driverA && intakeFrwd && !driverAhold) {
            intakeFrwd = false;
            driverAhold = true;
        } else if (!inputManager.driverA) {
            driverAhold = false;
        }

        if (inputManager.driverB) {
            intakeRev = true;
        } else {
            intakeRev = false;
        }
        //Turret & hood
        if (inputManager.opPOV != -1 && inputManager.opLeftTrigger < .85) {
            if(inputManager.opPOV == 0) {
                hoodState = HoodState.Raising;
            } else if (inputManager.opPOV == 90) {
                turretState = -1;
            } else if (inputManager.opPOV == 180) {
                hoodState = HoodState.Lowering;
            } else if (inputManager.opPOV == 270) {
                turretState = 1;
            }
        } else if (inputManager.opLeftTrigger > .85) {
            turretState = 2;
            hoodState = HoodState.Automatic;
        } else {
            turretState = 0;
            if (hoodState != HoodState.Zeroing) {
                hoodState = HoodState.Holding;
            }
        }

        //Manual Index
        if (inputManager.opY && indexerHold == 0) {
            isAdvancing = true;
            indexerHold++;
        } else if (!inputManager.opY && shootState == ShooterState.NotShooting) {
            isAdvancing = false;
            indexerHold = 0;
        }
        
        //shooter
        double triggerPressed = .85;
        if (inputManager.opRightTrigger > triggerPressed) {
            shooterOn = true;
        } else {
            shooterOn = false;
        }
    }

    public void update(double dt) {
        if (flywheelOn) {
            setFlywheel(14000);
        } else {
            setFlywheel(0);
        }

        switch (turretState) {
            case -1:
                turret.set(ControlMode.PercentOutput, .25);
                break;
            case 0:
                turret.set(ControlMode.PercentOutput, 0);
                break;
            case 1:
                turret.set(ControlMode.PercentOutput, -.25);
                break;
            case 2:
                alignShooter();
                break;
        }

        System.out.println(hood.getSelectedSensorPosition());
        System.out.println(hoodLimit.get());
        switch (hoodState) {
            case Raising:
                System.out.println("Hood up");
                setHood(0.1);
                break;
            case Holding:
               setHood(0);
                break;
            case Lowering:
                System.out.println("Hood down");
                setHood(-0.1);
                break;
            case Zeroing:
                System.out.println("Zeroing hood");
                zeroHood();
        }

        if (isAdvancing) {
            indexer.set(ControlMode.PercentOutput, 20);
            isAdvancing = hasAdvanced();
        } else {
            indexer.set(ControlMode.PercentOutput, 0);
        }
        intake();
        shoot();
    }

    public void intake() {
        switch (intakeState) {
            //Make sure that the intake is up, and off.
            case IntakeUp:
                intake.set(ControlMode.PercentOutput, 0);

                if (intakeFrwd) {
                    intakeState = IntakeState.IntakeDown;
                }
                break;
            //deploy the intake
            case IntakeDown:
            intake.set(ControlMode.PercentOutput, 100);

                if (indexerFull() && !intakeFrwd) {
                    intakeState = IntakeState.IntakeUp;
                } else if (intakeRev) {
                    intakeState = IntakeState.IntakeDownRev;
                }
                break;
            //reverse the intake, once it is deployed
            case IntakeDownRev:
                intake.set(ControlMode.PercentOutput, -100);
                 
                if (!intakeFrwd) {
                    intakeState = IntakeState.IntakeUp;
                } else if (!intakeRev) {
                    intakeState = IntakeState.IntakeDown;
                }
                break;
            case Indexing: //Unknown if this is needed
                break;
        }
    }

    public void shoot() {
        switch (shootState) {
            case NotShooting:
                if (shooterOn && flywheelUpToSpeed()) {
                    shootState = ShooterState.ShootBall;
                } else if (shooterOn && !flywheelUpToSpeed()) {
                    shootState = ShooterState.FlywheelOn;
                }
                break;
            case ShootBall:
                if (!shooterOn) {
                    shootState = ShooterState.NotShooting;
                    break;
                }
                uptake.set(ControlMode.PercentOutput, 100);
                if (indexerHasBalls()) {
                    shootState = ShooterState.LoadBall;
                } else if (!indexerHasBalls()) {
                    shootState = ShooterState.FlywheelOff;
                }
                break;
            case LoadBall:
                if (true) {
                    shootState = ShooterState.ShootBall;
                }
                break;
            case FlywheelOn:
                setFlywheel(100);
                if (flywheelUpToSpeed()) {
                    shootState = ShooterState.ShootBall;
                }
                break;
            case FlywheelOff:
                setFlywheel(0);
                shootState = ShooterState.NotShooting;
                break;
        }
    }

    public void autoIndex() {
        /*
        switch (indexerState) {
            //checks for the first ball intook
            case NoBalls:
                if(ballLimits[0].get() == true && intakeState == IntakeState.IntakeDown) {
                    indexerState = IndexerState.Rotate;
                }
                break;
            //rotates the indexer
            case Rotate:
                isAdvancing = true;
                if (hasAdvanced()) {
                    indexerState = IndexerState.StopRotating;
                    isAdvancing = false;
                }
                break;
            //stops rotating
            case StopRotating: 
                //no places left and uptake ready
                if (indexerFull() && hasPrepedBall()) {
                    indexerState = IndexerState.StopRotating;
                    break;
                }
                //If the indexer is not full and there is a ball in the intake pos, stop intakeing and ball not under uptake
                if ((!indexerFull() && ballLimits[0].get()) || (intakeState != IntakeState.IntakeDown && !ballLimits[3].get())) {
                    indexerState = IndexerState.Rotate;
                    break;
                }
                //ball in uptake pos and uptake empty
                if (!hasPrepedBall() && ballLimits[3].get()) {
                    indexerState = IndexerState.UptakeBall;
                    break;
                }
                break;
            //uptakes a ball
            case UptakeBall:
                uptake.set(ControlMode.PercentOutput, 100);
                if (hasPrepedBall()) {
                    indexerState = IndexerState.StopUptake;
                }
                break;
            //stops the uptake
            case StopUptake:
                if (ballLimits[0].get() ||ballLimits[1].get() || ballLimits[2].get() || ballLimits[3].get()) {
                    indexerState = IndexerState.Rotate;
                } else {
                    indexerState = IndexerState.NoBalls;
                }
                
                break;
        }
        */
    }

    private boolean indexerFull() {
        if (ballLimits[0].get() && ballLimits[1].get() && ballLimits[2].get() && ballLimits[3].get()) {
            return true;
        } else {
            return false;
        }
    }

    private boolean indexerHasBalls() {
        if (ballLimits[0].get() || ballLimits[1].get() || ballLimits[2].get() || ballLimits[3].get() || ballLimits[4].get()) {
            return true;
        } else {
            return false;
        }
    }

    private void setHood(double value) {
        switch(hoodControlState) {
            case PercentOutput:
                hood.set(ControlMode.PercentOutput, value);
                break;
            case MotionMagic:
                hood.set(ControlMode.Velocity, value);
                break;
            default:
                System.out.println("Using default case for flywheel set");
                hood.set(ControlMode.PercentOutput, value);
                break;
        }
    }

    private void setHoodControlState(HoodControlState newState) {
        if (hoodControlState == newState) {
            System.out.println("hoodControlState already equals " + newState);
            return;
        }
        hoodControlState = newState;
    }

    private void setFlywheel(double value) {
        switch(shooterControlState) {
            case PercentOutput:
                flywheel.set(ControlMode.PercentOutput, value);
                break;
            case VelocityPID:
                flywheel.set(ControlMode.Velocity, value);
                break;
            default:
                System.out.println("Using default case for flywheel set");
                flywheel.set(ControlMode.PercentOutput, value);
                break;
        }
    }

    private void setFlywheelState(ShooterControlState newState) {
        if (shooterControlState == newState) {
            System.out.println("ShooterControlState already equals " + newState);
            return;
        }
        shooterControlState = newState;
    }

    private boolean hasAdvanced() {
        return !indexerRotationLimit.get();
    }

    private boolean flywheelUpToSpeed() {
        return true;
    }

    private void updateLimelightData() {
        targetAngleHorizontal = tx.getDouble(Double.POSITIVE_INFINITY);
        System.out.println("Target ANgle Horizontal: " + targetAngleHorizontal);
        targetAngleVertical = ty.getDouble(Double.POSITIVE_INFINITY);
        targetAreaScalar = ta.getDouble(Double.POSITIVE_INFINITY);
        validTargets = tv.getDouble(Double.POSITIVE_INFINITY);
        System.out.println("Tv: " + validTargets);
    }

    private void printTurretEncoderData() {
        int selSenPos = turret.getSelectedSensorPosition(0);
		int pulseWidthWithoutOverflows = turret.getSensorCollection().getPulseWidthPosition() & 0xFFF;
		System.out.println("pulseWidPos:" + pulseWidthWithoutOverflows + "   =>    " + "selSenPos:" + selSenPos);
    }

    private void zeroHood() {
        if (!hoodLimit.get()) {
            hoodControlState = HoodControlState.PercentOutput;
            setHood(-0.1);
        } else {
            hood.setSelectedSensorPosition(0);
            hoodState = HoodState.Holding;
        }
    }

    private void alignShooter() {
        updateLimelightData();
        autoTurretState = TurretState.Tracking;
        if ((validTargets == 0 || validTargets == Double.POSITIVE_INFINITY) || targetAngleHorizontal == Double.POSITIVE_INFINITY) {
            System.out.println("Error Align Shooter, Either no targets or recieving default value");
        }
        /*
        if (validTargets == 0) {
            System.out.println("No targets in view");
            if (turret.getSelectedSensorPosition() != maximumRotation && !hasHitLeft) {
                turret.set(ControlMode.MotionMagic, maximumRotation);
                if (turret.getSelectedSensorPosition() == maximumRotation) {
                    hasHitLeft = true;
                }
            } else {
                turret.set(ControlMode.MotionMagic, -maximumRotation);
                if (turret.getSelectedSensorPosition() == -maximumRotation) {
                    System.out.println("ur dumb");
                }
            }
            
        } */else {
            //handle hood
            //double turretAngle = turret.getSelectedSensorPosition();
            if (targetAngleHorizontal > 0.0 - targetDeadband) {
                //spin one way
                turret.set(ControlMode.PercentOutput, .1);
            } else if (targetAngleHorizontal < 0.0 + targetDeadband) {
                turret.set(ControlMode.PercentOutput, -.1);
                //go the other way
            } else 
                turret.set(ControlMode.PercentOutput, 0.0);
            }
        }

    
    public enum TurretState {
        Tracking,
        Holding,
        ReadyToShoot,
        Manual;
    }

    public enum HoodState {
        Raising,
        Lowering,
        Holding,
        Automatic,
        Zeroing;
    }

    public enum HoodControlState {
        PercentOutput,
        MotionMagic,
        PositionPID;
    }

    public enum IndexerState {
        NoBalls,
        Rotate,
        StopRotating,
        UptakeBall,
        StopUptake;
    }

    public enum IntakeState {
        IntakeUp,
        IntakeDown,
        IntakeDownRev,
        Indexing; //We dont know if this will be needed, not currently called
    }

    public enum ShooterState {
        NotShooting,
        FlywheelOn,
        FlywheelOff,
        ShootBall,
        LoadBall;
    }

    public enum ShooterControlState {
        PercentOutput,
        VelocityPID;
    }
}