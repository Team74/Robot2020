package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Shooter {
    private BaseMotorController flywheel;
    private BaseMotorController turret;
    private BaseMotorController hood;
    private BaseMotorController intake;
    private BaseMotorController indexer;
    private BaseMotorController uptake;

    private DoubleSolenoid intakeArm;

    private DigitalInput uptakeLimit;
    private DigitalInput indexerRotationLimit;
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

    // private int shootProgress = 1;
    private int runUptake = 10;

    private int flywheelHold = 0;
    private int intakeHold = 0;
    private int indexerHold = 0;
    private int shootHold = 0;

    private TurretState autoTurretState = TurretState.Holding;

    public Shooter(RobotMap robotMap, InputManager inputManager, Vision vision) {
        this.flywheel = robotMap.flywheel;
        this.turret = robotMap.turret;
        this.hood = robotMap.hood;
        this.intake = robotMap.intake;
        this.indexer = robotMap.indexer;
        this.uptake = robotMap.uptake;

        this.intakeArm = robotMap.intakeArm;
        
        this.uptakeLimit = robotMap.uptakeLimit;
        this.indexerRotationLimit = robotMap.indexerRotationLimit;
        this.ballLimits = robotMap.ballLimit;

        this.inputManager = inputManager;
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
        /*if (inputManager.opA || inputManager.opB) {
            if (inputManager.opA && !inputManager.opB && intakeHold == 0) {
                intakeState = 1;
            } else if (!inputManager.opA && inputManager.opB && intakeHold == 0) {
                intakeState = -1;
            }
            intakeHold++;
        } else {
            intakeState = 0;
            intakeHold = 0;
        }*/

        if (inputManager.opA) {
            intakeFrwd = true;
        } else {
            intakeFrwd = false;
        }

        if (inputManager.opB) {
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

        // if (inputManager.opY && indexerHold == 0) {
        //     isAdvancing = true;
        //     indexerHold++;
        // } else if (!inputManager.opY && shootState == 0) {
        //     isAdvancing = false;
        //     indexerHold = 0;
        // }
        
        double triggerPressed = .85;
        if (inputManager.opRightTrigger > triggerPressed) {
            shooterOn = true;
        } else {
            shooterOn = false;
        }

        //shooter
        // int holdTime = 16;
        // if (inputManager.opRightTrigger > triggerIsPressed) {
        //   shootHold++;
        //   shootState = 0;
        //   if (shootHold > holdTime) {
        //     System.out.println("Fire all Balls");
        //     shootState = 2;
        //   }
        // } else {
        //   if (shootHold > 0 && shootHold < holdTime) {
        //     System.out.println("Fire one Ball");
        //     shootState = 1;
        //   } else if (shootHold > 0 && shootHold > holdTime) {
        //     System.out.println("Stopping Multifire");
        //     shootState = 0;
        //   }
        //   shootHold = 0;
        // }
    }

    public void update() {
        if (flywheelOn) {
           flywheel.set(ControlMode.PercentOutput, 10);
        } else {
           flywheel.set(ControlMode.PercentOutput, 0);
        }

        switch (turretState) {
            case -1:
                turret.set(ControlMode.PercentOutput, 100);
                break;
            case 0:
               turret.set(ControlMode.PercentOutput, 0);
                break;
            case 1:
                turret.set(ControlMode.PercentOutput, -100);
                break;
            case 2:
                alignShooter();
                break;
        }

        switch (hoodState) {
            case -1:
                hood.set(ControlMode.PercentOutput, 100);
                break;
            case 0:
               hood.set(ControlMode.PercentOutput, 0);
                break;
            case 1:
                hood.set(ControlMode.PercentOutput, -100);
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
        switch (intakeState) {
            case IntakeUp:
                intakeArm.set(Value.kReverse);
                intake.set(ControlMode.PercentOutput, 0);
                break;
            case IntakeDown:
                break;
            case Reverse:
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
        System.out.println(indexerState);
        System.out.println(hasAdvanced());
        System.out.println(hasPrepedBall());
        switch (indexerState) {
            //checks for the first ball intook
            case NoBalls:
                if(inputManager.driverA == true && intakeState == IntakeState.IntakeDown) {
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
                if ((!indexerFull() && inputManager.driverA) || (intakeState != IntakeState.IntakeDown && !inputManager.driverX)) {
                    indexerState = IndexerState.Rotate;
                    break;
                }
                //ball in uptake pos and uptake empty
                if (!hasPrepedBall() && inputManager.driverX) {
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
                if (inputManager.driverA || inputManager.driverB || inputManager.driverY || inputManager.driverX) {
                    indexerState = IndexerState.Rotate;
                } else {
                    indexerState = IndexerState.NoBalls;
                }
                
                break;
        }
    }

    private boolean indexerFull() {
        if (ballLimits[0].get() && ballLimits[1].get() && ballLimits[2].get() && ballLimits[3].get() && ballLimits[4].get()) {
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
        Reverse,
        Indexing; //We dont know if this will be needed
    }

    public enum ShooterState {
        NotShooting,
        FlywheelOn,
        FlywheelOff,
        ShootBall,
        LoadBall;
    }
}