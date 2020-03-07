/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drivebase.DriveState;
import frc.robot.Shooter.HoodState;
import frc.robot.Shooter.LimelightLEDState;
import frc.robot.Shooter.TurretState;
import frc.robot.autonomous.AutonRunner;
import frc.robot.autonomous.modes.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private ArrayList<Updateable> updateableObjects;

  public static RobotMap robotMap;
  public static InputManager inputManager;

  private Drivebase drivebase;
  private Climber climber;
  private Shooter shooter;
  private Vision mVision;

  private AutonRunner autonRunner;
  private Timer timer;

  @Override
  public void robotInit() {
    robotMap = RobotMap.getInstance();

    inputManager = InputManager.getInstance();
    drivebase = Drivebase.getInstance();
    shooter = Shooter.getInstance();
    climber = Climber.getInstance();

    shooter.setLimelightLEDS(LimelightLEDState.Off);

    autonRunner = AutonRunner.getInstance();
    timer = new Timer();


    updateableObjects = new ArrayList<>() {
      private static final long serialVersionUID = -99999;
      {
        add(inputManager);
        add(climber);
        add(drivebase);
        add(shooter);
      }
    };
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    // robotMap.gearShift.set(Value.kForward);
    drivebase.zeroGyro();
    drivebase.zeroDriveEncoders();
    shooter.setTurretState(TurretState.Zeroing);
    shooter.setHoodState(HoodState.Zeroing);
    timer.start();
    autonRunner.setAuton(new ShootInitalBalls());
    autonRunner.start();
  }

  @Override
  public void autonomousPeriodic() {
    //No handleInput in autonomous
    for(Updateable object : updateableObjects) {
      object.update(0.0);
      object.dashboard();
    }
  }

  @Override
  public void teleopInit() {
    autonRunner.stop();
    // robotMap.gearShift.set(Value.kForward);
    drivebase.setDriveState(DriveState.Teleop);
    robotMap.indexer.setSelectedSensorPosition(0);
  }


  @Override
  public void teleopPeriodic() {
    for(Updateable object : updateableObjects) {
      object.handleInput();
      object.update(0.0);
      object.dashboard();
    }
  }

  @Override
  public void testPeriodic() {

  }
}
