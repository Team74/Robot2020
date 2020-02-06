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
    mRobotMap.gearShiftLeft.set(Value.kForward);
    // mRobotMap.gearShiftLeft.set(Value.kReverse);
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
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


  private int LThold = 0;
  private int Lhold = 0;
  private boolean wheelDeploy = false;


  @Override
  public void teleopPeriodic() {
    mInputManager.update(0);
    shooter.handleInput();
    shooter.update();
    shooter.autoIndex();

    if (mInputManager.driverLeftBumper) {
       mRobotMap.gearShiftLeft.set(Value.kForward);
     } else {

     }
     if (mInputManager.driverRightBumper) {
       mRobotMap.gearShiftLeft.set(Value.kReverse);
     } else {

     }

    // climber.handleInput();
    // climber.update(); 

    // mRobotMap.driveRightBack.set(1);
    // mRobotMap.driveLeftBack.set(1);
   
    //Driver Controls
    mRobotMap.drive.arcadeDrive(mInputManager.driverLeftStickY, mInputManager.driverLeftStickX);
 /*
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

    
    if (mInputManager.opY) {
      System.out.println("Advance Indexer");
    }

    int holdTime = 16;
    double triggerIsPressed = .85;
    if (mInputManager.opRightTrigger > triggerIsPressed) {
      LThold++;
      if (LThold > holdTime) {
        System.out.println("Fire all Balls");
      }
    } else {
      if (LThold > 0 && LThold < holdTime) {
        System.out.println("Fire one Ball");
      }
      LThold = 0;
    }

    if (mInputManager.opLeftTrigger > triggerIsPressed) {
      System.out.println("Auto Aim");
    }

    if (mInputManager.opLeftBumper) {
      if (wheelDeploy && Lhold == 0) {
        System.out.println("Retract Wheel");
        wheelDeploy = false;
      } else if (Lhold == 0) {
        System.out.println("Deploy Wheel");
        wheelDeploy = true;
      }
      Lhold++;
    } else {
      Lhold = 0;
    }
    */
  }

  @Override
  public void testPeriodic() {
  }
}
