/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import jdk.internal.util.xml.impl.Input;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  ArrayList<Updateable> updateableObjects = new ArrayList<>();

  static RobotMap robotMap;
  static InputManager inputManager;

  static Drivebase drivebase;
  static Climber climber;
  static Shooter shooter;
  static Vision mVision;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    updateableObjects.add(inputManager);
    
    robotMap = RobotMap.getInstance();
    inputManager = InputManager.getInstance();

    drivebase = Drivebase.getInstance();
    updateableObjects.add(drivebase);
    climber = Climber.getInstance();
    updateableObjects.add(climber);
    mVision = Vision.getInstance();
    shooter = Shooter.getInstance();
    updateableObjects.add(shooter);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    robotMap.gearShift.set(Value.kForward);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    robotMap.gearShift.set(Value.kForward);
  }


  @Override
  public void teleopPeriodic() {
    for(Updateable object : updateableObjects) {
      object.update(0.0);
      object.dashboard();
    }

    shooter.handleInput();
    shooter.autoIndex();
  }

  @Override
  public void testPeriodic() {

  }
}
