package frc.robot;

import java.util.HashMap;

import javax.management.RuntimeErrorException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Shooter implements Updateable {
    private static Shooter kInstance = null;

    private final RobotMap robotMap;

    private BaseMotorController intake, uptake;
    private TalonSRX turret, hood, flywheelMaster, indexer;
    private VictorSPX flywheelFollower;

    private DigitalInput hoodLimit, turretLimit, indexerRotationLimit;
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
    private boolean driverBhold = false;
    private boolean driverYhold = false;
    private int indexerHold = 0;

    private boolean isAdvancingRev = false;

    private boolean hasHitLeft = false;
    private final double turretAlignmentDeadband = 0.05;

    private TurretState turretState = TurretState.Zeroing;
    private TurretControlState turretControlState = TurretControlState.PercentOutput;

    private int ballCounter = 0;

    private NetworkTable limelight;
    private NetworkTableEntry tx, ty, ta, tv;
    private boolean isLedOn = false, ledToggling = false;

    //-27 to 27 degrees
    private double targetAngleHorizontal;
    //-20.5 to 20.5 degrees
    private double targetAngleVertical;
    private double targetAreaScalar;
    //0 no valid targets, 1 valid target
    private double validTargets;

    public Shooter() {
        robotMap = RobotMap.getInstance();

        flywheelMaster = robotMap.flywheelMaster;
        flywheelFollower = robotMap.flywheelFollower;
        turret = robotMap.turret;
        hood = robotMap.hood;
        intake = robotMap.intake;
        indexer = robotMap.indexer;
        uptake = robotMap.uptake;
        
        hoodLimit = robotMap.hoodLimit;
        turretLimit = robotMap.turretLimit;
        indexerRotationLimit = robotMap.indexerRotationCheck;
        ballLimits = robotMap.ballLimit;

        inputManager = Robot.inputManager;

        limelight = robotMap.limelight;

        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
        tv = limelight.getEntry("tv");
    }

    public static Shooter getInstance() {
        if (kInstance == null) {
            kInstance = new Shooter();
        } 
        return kInstance;
    }

    public void dashboard() {
        SmartDashboard.putBoolean("Gear", Robot.inputManager.opY);
        SmartDashboard.putNumber("Indexer Current", indexer.getSupplyCurrent());

        SmartDashboard.putBoolean("Index 1", Robot.robotMap.ballLimit[0].get());
        SmartDashboard.putBoolean("Index 2", Robot.robotMap.ballLimit[1].get());
        SmartDashboard.putBoolean("Index 3", Robot.robotMap.ballLimit[2].get());
        SmartDashboard.putBoolean("Index 4", Robot.robotMap.ballLimit[3].get());
        SmartDashboard.putBoolean("Index 5", Robot.robotMap.ballLimit[4].get());
        
        SmartDashboard.putBoolean("Flywheel", flywheelOn);
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

        //Intake
        //Fix/Clean up Intake
        if (inputManager.driverA && !driverAhold) {
            intakeState = IntakeState.IntakeDown;
            driverAhold = true;
        } else if (!inputManager.driverA) {
            driverAhold = false;
        }

        if (inputManager.driverB && !driverBhold) {
            intakeState = IntakeState.IntakeDownRev;
            driverBhold = true;
        } else if (!inputManager.driverB) {
            driverBhold = false;
        }

        if (!inputManager.driverA && !inputManager.driverB) {
            intakeState = IntakeState.IntakeUp;
        }
        //End Intake Call

        //Turret & hood
        if (inputManager.opPOV != -1 && inputManager.opLeftTrigger < .85) {
            if(inputManager.opPOV == 0) {
                hoodState = HoodState.Raising;
            } else if (inputManager.opPOV == 90) {
                turretState = TurretState.RightSpin;
            } else if (inputManager.opPOV == 180) {
                hoodState = HoodState.Lowering;
            } else if (inputManager.opPOV == 270) {
                turretState = TurretState.LeftSpin;
            }
        } else if (inputManager.opLeftTrigger > .85) {
            // hoodState = HoodState.Tracking;
            turretState = TurretState.Tracking;
        } else {
            if (turretState != TurretState.Zeroing) {
                turretState = TurretState.Holding;
            }
            if (hoodState != HoodState.Zeroing) {
                hoodState = HoodState.Holding;
            }
        }
        //Limelight led control
        if (Robot.inputManager.driverX && !isLedOn && !ledToggling) {
            isLedOn = true;
            ledToggling = true;
        } else if(Robot.inputManager.driverX && isLedOn && !ledToggling) {
            isLedOn = false;
            ledToggling = true;
        } else if (!Robot.inputManager.driverX) {
            ledToggling = false;
        }

        //Manual Index
        if (inputManager.opY && indexerHold == 0) {
            isAdvancing = true;
            indexerHold++;
        } else if (!inputManager.opY) {
            isAdvancing = false;
            indexerHold = 0;
        }

        if (inputManager.opB && indexerHold == 0) {
            isAdvancingRev = true;
            indexerHold++;
        } else if (!inputManager.opB) {
            isAdvancingRev = false;
            indexerHold = 0;
        }

        //shooter
        double triggerPressed = .85;
        if (inputManager.opRightTrigger > triggerPressed) {
            shooterOn = true;
        } else {
            shooterOn = false;
        }

        // if (inputManager.driverY && !driverYhold) {
        //     ballCounter--;
        //     driverYhold = true;
        // } else if (!inputManager.driverY && driverYhold) {
        //     driverYhold = false;
        // }
    }

    public void update(double dt) {
        handleInput();
        updateLimelightData();
        zeroTurretEncoder();
        zeroHoodEncoder();

        if(isLedOn) {
            setLimelightLEDS(LimelightLEDState.On);
        } else {
            setLimelightLEDS(LimelightLEDState.Off);
        }

        // System.out.println("Turret position " + turret.getSelectedSensorPosition(0));
        // System.out.println("Turret Velcity " + turret.getSelectedSensorVelocity(0));
        // System.out.println("Turret Limit: " + turretLimit.get());
        System.out.println("Hood position " + hood.getSelectedSensorPosition(0));
        // System.out.println("Target Angle Vertical " + targetAngleVertical);
        // System.out.println("Hood Velcity " + hood.getSelectedSensorVelocity(0)); 
        System.out.println(hoodLimit.get());
        // System.out.println(ballCounter);
        System.out.println("Flywheel Speed: "+ flywheelMaster.getSelectedSensorVelocity(0));
        System.out.println("Index Pose Limit" + indexerRotationLimit.get());

        index();
        //autoIndex();
        hood();
        turret();
        intake();
        shoot();

        // if (indexer.getSupplyCurrent() < Double.POSITIVE_INFINITY) {
        //     indexer.set(ControlMode.PercentOutput, 0);
        // }
    }

    public void index() {
        if (isAdvancing && !isAdvancingRev) {
            indexer.set(ControlMode.PercentOutput, -.3);
            //isAdvancing = hasAdvanced();
        } else if (!isAdvancing && isAdvancingRev) {
            indexer.set(ControlMode.PercentOutput, .3);
        } else {
            indexer.set(ControlMode.PercentOutput, 0);
        }
    }

    public void hood() {
        switch (hoodState) {
            case Raising:
                // System.out.println("Hood up");
                setHoodControlState(HoodControlState.PercentOutput);
                hoodControlState = HoodControlState.PercentOutput;
                setHood(.2);
                break;
            case Holding:
                setHoodControlState(HoodControlState.PercentOutput);
                setHood(0);
                break;
            case Lowering:
                // System.out.println("Hood down")
                setHoodControlState(HoodControlState.PercentOutput);
                setHood(-0.2);
                break;
            case Zeroing:
                // System.out.println("Zeroing hood");
                zeroHood();
                break;
            case Tracking:
                alignHood();
                break;
            default:
                break;
        }
    }

    public void turret() {
        // System.out.println("Turret State: " + turretState);
        switch (turretState) {
            case RightSpin:
                setTurret(-.5);
                break;
            case Holding:
                setTurret(0);
                break;
            case LeftSpin:
                setTurret(.5);
                break;
            case Tracking:
                alignTurret();
                break;
            case Zeroing:
                zeroTurret();
                break;
            default:
                break;
        }
    }

    public void intake() {
        switch (intakeState) {
            //Make sure that the intake is up, and off.
            case IntakeUp:
                intake.set(ControlMode.PercentOutput, 0);
                break;
            //deploy the intake
            case IntakeDown:
            intake.set(ControlMode.PercentOutput, 1.0);
                break;
            //reverse the intake, once it is deployed
            case IntakeDownRev:
                intake.set(ControlMode.PercentOutput, -1.0);
                break;
            case Indexing: //Unknown if this is needed
                break;
        }
    }

    public void shoot() {
        switch (shootState) {
            case NotShooting:
                if (shooterOn && flywheelUpToSpeed(28000)) {
                    shootState = ShooterState.ShootBall;
                } else if (shooterOn && !flywheelUpToSpeed(28000)) {
                    shootState = ShooterState.FlywheelOn;
                } else if (!shooterOn) {
                    shootState = ShooterState.FlywheelOff;
                }
                break;
            case ShootBall:
                if (!shooterOn) {
                    shootState = ShooterState.NotShooting;
                    uptake.set(ControlMode.PercentOutput, 0.0);
                    break;
                } else if (shooterOn) {
                    if(!indexerRotationLimit.get()) {
                        uptake.set(ControlMode.PercentOutput, 1.0);
                    } else {
                        uptake.set(ControlMode.PercentOutput, 0.0);
                    }
                }
                // ballCounter--;
                // if (indexerHasBalls()) {
                //     shootState = ShooterState.NotShooting;
                // } else if (!indexerHasBalls()) {
                //     shootState = ShooterState.FlywheelOff;
                // }
                break;
            case LoadBall:  
                if (true) {
                    shootState = ShooterState.NotShooting;
                }
                break;
            case FlywheelOn:
                setFlywheel(27000);
                if (flywheelUpToSpeed(28000)) {
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
        switch (indexerState) {
            //checks for the first ball intook
            case NoBalls:
                if(ballLimits[0].get()) {
                    ballCounter++;
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
                if (indexerFull()) {
                    indexerState = IndexerState.StopRotating;
                    break;
                } else if ((!indexerFull() && ballLimits[0].get())) { //If the indexer is not full and there is a ball in the intake pos, stop intakeing and ball not under uptake
                    indexerState = IndexerState.Rotate;
                    ballCounter++;
                    break;
                } 
                if (indexerFull() && ballLimits[0].get() && ballCounter < 5) {
                    ballCounter++;
                }
                if (shooterOn && !ballLimits[1].get() && indexerHasBalls()) {
                    indexerState = IndexerState.Rotate;
                }
                break;
            //uptakes a ball
            case UptakeBall:
            //     uptake.set(ControlMode.PercentOutput, 100);
            //     if (hasPrepedBall()) {
            //         indexerState = IndexerState.StopUptake;
            //     }
                break;
            // //stops the uptake
            case StopUptake:
            //     if (ballLimits[0].get() ||ballLimits[1].get() || ballLimits[2].get() || ballLimits[3].get()) {
            //         indexerState = IndexerState.Rotate;
            //     } else {
            //         indexerState = IndexerState.NoBalls;
            //     }
                
                break;
            case Manual:
                break;
            default:
                break;
        }
    }

    private boolean indexerFull() {
        if (ballLimits[0].get() && ballLimits[1].get() /*&& ballLimits[2].get() && ballLimits[3].get() && ballLimits[4].get()*/) {
            return true;
        } else {
            return false;
        }
    }

    private boolean indexerHasBalls() {
        if (ballCounter > 0/*ballLimits[0].get() || ballLimits[1].get() || ballLimits[2].get() || ballLimits[3].get() || ballLimits[4].get()*/) {
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
                hood.set(ControlMode.MotionMagic, (int)value);
                break;
            case PositionPID:
                hood.set(ControlMode.Position, (int)value);
                break;
            default:
                System.out.println("Using default case for hood set");
                hood.set(ControlMode.PercentOutput, value);
                break;
        }
    }

    public void setHoodControlState(HoodControlState newState) {
        if (hoodControlState == newState) {
            // System.out.println("hoodControlState already equals " + newState);
            return;
        }
        hoodControlState = newState;
    }

    public void setHoodState(HoodState newState) {
        if (hoodState == newState) {
            // System.out.println("hoodState already equals " + newState);
            return;
        }
        hoodState = newState;        
    }

    private void setTurret(double value) {
        int currentPose = turret.getSelectedSensorPosition(0);
        if (turretState != TurretState.Zeroing) {
            switch (turretControlState) {
                case PercentOutput:
                    if (currentPose < Constants.kTurretMinimumRotation && value < 0) {
                        turret.set(ControlMode.PercentOutput, 0.0);
                        break;
                    } else if (currentPose > Constants.kTurretMaxRotation && value > 0) {
                        turret.set(ControlMode.PercentOutput, 0.0);
                        break;
                    } else {
                        turret.set(ControlMode.PercentOutput, value);
                        break;
                    }
                case MotionMagic:
                    turret.set(ControlMode.MotionMagic, value);
                    break;
                default:
                    break;
            }
        } else if (turretState == TurretState.Zeroing) {
            turret.set(ControlMode.PercentOutput, value);
        }
    } 

    public void setTurretControlState(TurretControlState newState) {
        if (turretControlState == newState) {
            // System.out.println("turretControlState already equals " + newState);
            return;
        }
        turretControlState = newState;
    }

    public void setTurretState(TurretState newState) {
        if (turretState == newState) {
            // System.out.println("turretState already equals " + newState);
            return;
        }
        turretState = newState;  
    }

    private void setFlywheel(double value) {
        switch(shooterControlState) {
            case PercentOutput:
                flywheelMaster.set(ControlMode.PercentOutput, value);
                break;
            case VelocityPID:
                flywheelMaster.set(ControlMode.Velocity, (int)value);
                break;
            default:
                System.out.println("Using default case for flywheel set");
                flywheelMaster.set(ControlMode.PercentOutput, (int)value);
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
        return indexerRotationLimit.get();
    }

    private boolean flywheelUpToSpeed(int targetSpeed) {
        int currentSpeed = flywheelMaster.getSelectedSensorVelocity(0);
        return (currentSpeed >= (targetSpeed - 1000) && currentSpeed <= (targetSpeed + 1000));
    }

    private void updateLimelightData() {
        targetAngleHorizontal = tx.getDouble(Double.POSITIVE_INFINITY);
        // System.out.println("Target ANgle Horizontal: " + targetAngleHorizontal);
        targetAngleVertical = ty.getDouble(Double.POSITIVE_INFINITY);
        targetAreaScalar = ta.getDouble(Double.POSITIVE_INFINITY);
        validTargets = tv.getDouble(Double.POSITIVE_INFINITY);
        // System.out.println("Tv: " + validTargets);
    }

    private void setLimelightLEDS(LimelightLEDState newState) {
        switch(newState) {
            case On:
                limelight.getEntry("ledMode").setNumber(3);
                break;
            case Off:
                limelight.getEntry("ledMode").setNumber(1);
                break;
            case Blinking:
                limelight.getEntry("ledMode").setNumber(2);
                break;
            default:
                limelight.getEntry("ledMode").setNumber(3);
                break;
        }
    }

    private void printTurretEncoderData() {
        int selSenPos = turret.getSelectedSensorPosition(0);
		int pulseWidthWithoutOverflows = turret.getSensorCollection().getPulseWidthPosition() & 0xFFF;
		// System.out.println("pulseWidPos:" + pulseWidthWithoutOverflows + "   =>    " + "selSenPos:" + selSenPos);
    }

    private void zeroHood() {
        if (!hoodLimit.get()) {
            hoodControlState = HoodControlState.PercentOutput;
            setHood(-0.25);
        } else {
            setHood(0.0);
            hoodState = HoodState.Holding;
        }
    }

    private void zeroHoodEncoder() {
        if (turretLimit.get()) {
            hood.setSelectedSensorPosition(0, 0, 30);
        }
    }

    private void zeroTurret() {
        if (!turretLimit.get()) {
            setTurret(-0.20);
        } else {
            setTurret(0);
            turretState = TurretState.Holding;
        }
    }

    private void zeroTurretEncoder() {
        if (turretLimit.get()) {
            turret.setSelectedSensorPosition(0, 0, 30);
        }
    }

    private void alignTurret() {
        if ((validTargets == 0 || validTargets == Double.POSITIVE_INFINITY) || targetAngleHorizontal == Double.POSITIVE_INFINITY) {
            System.out.println("Error Align Shooter, Either no targets or recieving default value");
        }
        
        if (validTargets == 0) {
            System.out.println("No targets in view");
            setTurretControlState(TurretControlState.MotionMagic);
            if (turret.getSelectedSensorPosition() != Constants.kTurretMaxRotation && !hasHitLeft) {
                setTurret(Constants.kTurretMaxRotation);
                if (turret.getSelectedSensorPosition() == Constants.kTurretMaxRotation) {
                    hasHitLeft = true;
                }
            } else {
                setTurret(Constants.kTurretMinimumRotation);
                if (turret.getSelectedSensorPosition() == Constants.kTurretMinimumRotation) {
                    System.out.println("ur dumb");
                }
            }
            setTurretControlState(TurretControlState.PercentOutput);
        } else {
            if (Math.abs(targetAngleHorizontal) > turretAlignmentDeadband) {
                System.out.println("Porportional Control");
                double spinPower = targetAngleHorizontal * (-1.0/54.0);
                if (Math.abs(spinPower) < 0.075) {
                    spinPower = Math.copySign(0.075, spinPower);
                }
                setTurret(spinPower);
            } else 
                setTurret(0.0);
            }
    }

    private void alignHood() {
        double hoodPositionTicks = (-1250 * targetAngleVertical) + 12500;
        if (hoodPositionTicks < Constants.kHoodMinimumHeight) {
            hoodPositionTicks = Constants.kHoodMinimumHeight;
        } else if (hoodPositionTicks > Constants.kHoodMaxHeight) {
            hoodPositionTicks = Constants.kHoodMaxHeight;
        }
        setHood(hoodPositionTicks);
    }

    public enum LimelightLEDState {
        On,
        Off,
        Blinking;
    }

    
    public enum TurretState {
        Tracking,
        Holding,
        ReadyToShoot,
        Manual,
        LeftSpin,
        RightSpin,
        Zeroing;
    }

    public enum HoodState {
        Raising,
        Lowering,
        Manual,
        Holding,
        Tracking,
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
        Manual,
        StopRotating,
        UptakeBall,
        StopUptake;
    }

    public enum IntakeState {
        IntakeUp,
        IntakeDown,
        IntakeDownRev,
        Manual,
        Indexing; //We dont know if this will be needed, not currently called
    }

    public enum ShooterState {
        NotShooting,
        Manual,
        FlywheelOn,
        FlywheelOff,
        ShootBall,
        LoadBall;
    }

    public enum ShooterControlState {
        PercentOutput,
        VelocityPID;
    }

    public enum TurretControlState {
        PercentOutput,
        MotionMagic;
    }
}