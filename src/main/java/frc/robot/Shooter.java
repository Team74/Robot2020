package frc.robot;

import java.util.HashMap;

import javax.management.RuntimeErrorException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.DigitalInput;

public class Shooter {
    private BaseMotorController flywheel;
    private BaseMotorController turret;
    private BaseMotorController hood;
    private BaseMotorController intake;
    private BaseMotorController indexer;
    private BaseMotorController uptake;

    private DigitalInput uptakeLimit;
    private DigitalInput indexerRotationLimit;
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
    private int intakeState = 0;
    private int turretState = 0;
    private int hoodState = 0;
    private boolean isAdvancing = false;
    private int shootState = 0;
    private int indexerState = 0;

    private int shootProgress = 1;
    private int runUptake = 10;

    private int flywheelHold = 0;
    private int intakeHold = 0;
    private int indexerHold = 0;
    private int shootHold = 0;

    private int maximumRotation = 891324;
    private boolean hasHitLeft = false;

    private TurretState autoTurretState = TurretState.Holding;

    public Shooter(RobotMap robotMap, InputManager inputManager, Vision vision) {
        this.flywheel = robotMap.flywheel;
        this.turret = robotMap.turret;
        this.hood = robotMap.hood;
        this.intake = robotMap.intake;
        this.indexer = robotMap.indexer;
        this.uptake = robotMap.uptake;
        this.inputManager = inputManager;
        this.vision = vision;
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
        if (inputManager.opA || inputManager.opB) {
            if (inputManager.opA && !inputManager.opB && intakeHold == 0) {
                intakeState = 1;
            } else if (!inputManager.opA && inputManager.opB && intakeHold == 0) {
                intakeState = -1;
            }
            intakeHold++;
        } else {
            intakeState = 0;
            intakeHold = 0;
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

        if (inputManager.opY && indexerHold == 0) {
            isAdvancing = true;
            indexerHold++;
        } else if (!inputManager.opY && shootState == 0) {
            isAdvancing = false;
            indexerHold = 0;
        }

        //shooter
        int holdTime = 16;
        double triggerIsPressed = .85;
        if (inputManager.opRightTrigger > triggerIsPressed) {
          shootHold++;
          shootState = 0;
          if (shootHold > holdTime) {
            System.out.println("Fire all Balls");
            shootState = 2;
          }
        } else {
          if (shootHold > 0 && shootHold < holdTime) {
            System.out.println("Fire one Ball");
            shootState = 1;
          } else if (shootHold > 0 && shootHold > holdTime) {
            System.out.println("Stopping Multifire");
            shootState = 0;
          }
          shootHold = 0;
        }
    }

    public void update() {
        System.out.println("shoot State " + shootState);
        System.out.println("shoot progress " + shootProgress);
        if (flywheelOn) {
           flywheel.set(ControlMode.PercentOutput, 10);
        } else {
           flywheel.set(ControlMode.PercentOutput, 0);
        }

        switch (intakeState) {
            case -1:
                intake.set(ControlMode.PercentOutput, 100);
                break;
            case 0:
                intake.set(ControlMode.PercentOutput, 0);
                break;
            case 1:
                intake.set(ControlMode.PercentOutput, -100);
                break;
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
            isAdvancing = !hasAdvanced();
        } else {
            indexer.set(ControlMode.PercentOutput, 0);
        }

        if (shootState > 0) {
            startShooting();
            if (shootProgress == 1) {
                if (hasPrepedBall()) {
                    shootProgress++;
                } else {
                    isAdvancing = true;
                }
            } 
            if (shootProgress == 2) {
                if (runUptake != 0) {
                  uptake.set(ControlMode.PercentOutput, 45);
                  runUptake--;
                } else {
                  uptake.set(ControlMode.PercentOutput, 0);
                  runUptake = 10;
                  shootProgress = 1;
                  if (shootState == 1) {
                      shootState = 0;
                  }
                }
            }
        } else {
            uptake.set(ControlMode.PercentOutput, 0);
        }
    }

    public void autoIndex() {

    }

    private void startShooting() {
        flywheelOn = true;
    }

    private boolean hasAdvanced() {
        return !indexerRotationLimit.get();
    }

    private boolean hasPrepedBall() {
        return !uptakeLimit.get();
    }

    private void alignShooter() {
        autoTurretState = TurretState.Tracking;
        HashMap<String, Double> aimingData = VisionProcessing.calculateDistanceAngle(vision.getData(), highPort);
        if (aimingData == null) {
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
        } else {
            //handle hood
            double turretAngle = turret.getSelectedSensorPosition();
            double targetPose = turret.getSelectedSensorPosition() + (toEncoderTicks(aimingData.get("angle")) * -1);
            turret.set(ControlMode.MotionMagic, targetPose);
            if (turret.getSelectedSensorPosition() == targetPose) {
                autoTurretState = TurretState.ReadyToShoot;
            } 
        }
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
}