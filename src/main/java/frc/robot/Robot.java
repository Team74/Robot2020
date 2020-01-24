/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

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

  RobotMap mRobotMap;
  InputManager mInputManager;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    mRobotMap = RobotMap.getInstance();
    mInputManager = InputManager.getInstance();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
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
  public void teleopPeriodic() {
    mInputManager.update(0);

    //Driver Controls
    mRobotMap.drive.arcadeDrive(mInputManager.driverLeftStickY, mInputManager.driverRightStickX);

    if (mInputManager.driverPOV == 0) {
      System.out.println("Climber Extend");
    } else if (mInputManager.driverPOV == 90) {
      System.out.println("Climber Right");
    } else if (mInputManager.driverPOV == 180) {
      System.out.println("Climber Retract");
    } else if (mInputManager.driverPOV == 270) {
      System.out.println("Climber Left");
    }

    //Operator Controls

    if (mInputManager.opLeftStickX > .5) {
      System.out.println("Wheel Spin Right");
    } else if (mInputManager.opLeftStickX < -.5) {
      System.out.println("Wheel Spin Left");
    }

    if (mInputManager.opPOV == 0) {
      System.out.println("Hood Up");
    } else if (mInputManager.opPOV == 90) {
      System.out.println("Shooter Turn Right");
    } else if (mInputManager.opPOV == 180) {
      System.out.println("Hood Down");
    } else if (mInputManager.opPOV == 270) {
      System.out.println("Shooter Turn Left");
    }

    if (mInputManager.opA && !mInputManager.opB) {
      System.out.println("Intake Balls");
    } else if (!mInputManager.opA && mInputManager.opB) {
      System.out.println("Rev. Intake Balls");
    }

    if (mInputManager.opX) {
      System.out.println("Flywheel");
    }
    if (mInputManager.opY) {
      System.out.println("Advance Indexer");
    }

    
  }

  @Override
  public void testPeriodic() {
  }
}
