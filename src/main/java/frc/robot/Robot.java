/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

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
  Climber climber;
  Shooter shooter;
  Vision mVision;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    mRobotMap = RobotMap.getInstance();
    mInputManager = InputManager.getInstance();

    climber = new Climber(mInputManager, mRobotMap);
    mVision = Vision.getInstance();
    shooter = new Shooter(mRobotMap, mInputManager, mVision);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    mRobotMap.gearShift.set(Value.kForward);
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
    shooter.handleInput();
    shooter.update();
    shooter.autoIndex();
    System.out.println(mRobotMap.intake.getOutputCurrent());

    if (mInputManager.driverLeftBumper) {
       mRobotMap.gearShift.set(Value.kForward);
     } else {

     }
     if (mInputManager.driverRightBumper) {
       mRobotMap.gearShift.set(Value.kReverse);
     }

     if (mInputManager.driverTriggerRight > .85) {
      mInputManager.driveScaler = .5;
     } else {
      mInputManager.driveScaler = 1;
     }

    // climber.handleInput();
    // climber.update(); 

    // mRobotMap.driveRightBack.set(1);
    // mRobotMap.driveLeftBack.set(1);
   
    //Driver Controls
    mRobotMap.drive.arcadeDrive(mInputManager.driverLeftStickY, mInputManager.driverRightStickX);
  }

  @Override
  public void testPeriodic() {
  }
}
