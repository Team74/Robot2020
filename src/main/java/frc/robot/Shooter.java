package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Shooter implements Updateable {
    private static Shooter kInstance = null;

    private BaseMotorController flywheel, turret, hood, intake, indexer, uptake;

    private DoubleSolenoid intakeArm;

    private DigitalInput uptakeLimit, indexerRotationLimit;
    private DigitalInput[] ballLimits;

    private InputManager inputManager;

    private Boolean flywheelOn = false;
    private IntakeState intakeState = IntakeState.IntakeUp;
    private int turretState = 0;
    private int hoodState = 0;
    private boolean isAdvancing = false;
    private ShooterState shootState = ShooterState.NotShooting;
    private Boolean shooterOn = false;
    private IndexerState indexerState = IndexerState.NoBalls;

    private boolean intakeFrwd = false;
    private boolean intakeRev = false;

    private int flywheelHold = 0;
    private boolean driverAhold = false;
    private int indexerHold = 0;

    private TurretState autoTurretState = TurretState.Holding;

    public Shooter() {
        flywheel = Robot.robotMap.flywheel;
        turret = Robot.robotMap.turret;
        hood = Robot.robotMap.hood;
        intake = Robot.robotMap.intake;
        indexer = Robot.robotMap.indexer;
        uptake = Robot.robotMap.uptake;

        intakeArm = Robot.robotMap.intakeArm;
        
        uptakeLimit = Robot.robotMap.uptakeLimit;
        indexerRotationLimit = Robot.robotMap.indexerRotationLimit;
        ballLimits = Robot.robotMap.ballLimit;

        inputManager = Robot.inputManager;
    }

    public static Shooter getInstance() {
        if (kInstance == null) {
            kInstance = new Shooter();
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
                hoodState = -1;
            } else if (inputManager.opPOV == 90) {
                turretState = -1;
            } else if (inputManager.opPOV == 180) {
                hoodState = 1;
            } else if (inputManager.opPOV == 270) {
                turretState = 1;
            }
        } else if (inputManager.opLeftTrigger > .85) {
            turretState = 2;
            hoodState = 2;
        } else {
            turretState = 0;
            hoodState = 0;
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
           flywheel.set(ControlMode.PercentOutput, 10);
        } else {
           flywheel.set(ControlMode.PercentOutput, 0);
        }

        switch (turretState) {
            case -1:
                turret.set(ControlMode.PercentOutput, 25);
                break;
            case 0:
                turret.set(ControlMode.PercentOutput, 0);
                break;
            case 1:
                turret.set(ControlMode.PercentOutput, -25);
                break;
            case 2:
                alignShooter();
                break;
        }

        switch (hoodState) {
            case -1:
                hood.set(ControlMode.PercentOutput, 25);
                break;
            case 0:
               hood.set(ControlMode.PercentOutput, 0);
                break;
            case 1:
                hood.set(ControlMode.PercentOutput, -25);
                break;
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
        System.out.println(intakeState);
        switch (intakeState) {
            //Make sure that the intake is up, and off.
            case IntakeUp:
                intakeArm.set(Value.kReverse);
                intake.set(ControlMode.PercentOutput, 0);

                if (intakeFrwd) {
                    intakeState = IntakeState.IntakeDown;
                }
                break;
            //deploy the intake
            case IntakeDown:
                intakeArm.set(Value.kForward);
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
                if (uptakeLimit.get()) {
                    shootState = ShooterState.ShootBall;
                }
                break;
            case FlywheelOn:
                flywheel.set(ControlMode.PercentOutput, 100);
                if (flywheelUpToSpeed()) {
                    shootState = ShooterState.ShootBall;
                }
                break;
            case FlywheelOff:
                flywheel.set(ControlMode.PercentOutput, 0);
                shootState = ShooterState.NotShooting;
                break;
        }
    }

    public void autoIndex() {
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

    private boolean hasAdvanced() {
        return !indexerRotationLimit.get();
    }

    private boolean hasPrepedBall() {
        return !uptakeLimit.get();
    }

    private boolean flywheelUpToSpeed() {
        return true;
    }

    private void alignShooter() {
        autoTurretState = TurretState.Tracking;
        //if ()
        //If no target in screen
            //Turn to left limit
            //Check for target in screen periodically
                //If no target found turn to right
                //Check for target in screen periodically
        //If a target is in view
            //calculate where we need to go
            //Go there
        //Once locked onto target, set state to holding and signal that we are prepared to shoot
        //Possible use angelo backups
        turret.set(ControlMode.PercentOutput, 0);
        System.out.println("align  shooter");
    }

    public enum TurretState {
        Tracking,
        Holding,
        ReadyToShoot,
        Manual;
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
}