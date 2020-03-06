package frc.robot;

/*
Class that handles *human* input; for sensors, see SensorManager. Since the various TeleopMaster subclasses are the only thing that should be touching this class, it can get updated from there.
*/

import frc.robot.Updateable;

//import frc.lib.utils.Utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;


public class InputManager implements Updateable {
    public static InputManager kInstance = null; 

    private XboxController mController0 = new XboxController(0);
    private XboxController mController1 = new XboxController(1);

    //driver buttons 

    public double driverLeftStickY = 0;
    public double driverLeftStickX = 0;
    public double driverRightStickY = 0;
    public double driverRightStickX = 0;

    public boolean driverA = false;
    public boolean driverB = false;
    public boolean driverX = false;
    public boolean driverY = false;

    public boolean driverLeftBumper = false;
    public boolean driverRightBumper = false;

    public double driverTriggerLeft = 0;
    public double driverTriggerRight = 0;

    public int driverPOV = -1;

    //operator buttons
    public double opLeftStickY = 0;
    public double opLeftStickX = 0;
    public double opRightStickY = 0;
    public double opRightStickX = 0;


    public boolean opA = false;
    public boolean opB = false;
    public boolean opX = false;
    public boolean opY = false;

    public boolean opLeftBumper = false;
    public boolean opRightBumper = false;

    public double opRightTrigger = 0;
    public double opLeftTrigger = 0;

    public int opPOV = -1;

    public static InputManager getInstance() {
        if (kInstance == null) {
            kInstance = new InputManager();
        }
        return kInstance;
    }

    /**
     * Nothing here, just to statisfy updateable interface
     */
    public void handleInput() {

    }

    public void update(double dt) {
        driverLeftStickY = mController0.getY(Hand.kLeft);
        driverLeftStickX = mController0.getX(Hand.kLeft);
        driverRightStickY = mController0.getY(Hand.kRight);
        driverRightStickX = mController0.getX(Hand.kRight);

        driverA = mController0.getAButton();
        driverB = mController0.getBButton();
        driverX = mController0.getXButton();
        driverY = mController0.getYButton();

        driverTriggerLeft = mController0.getTriggerAxis(Hand.kLeft);
        driverTriggerRight = mController0.getTriggerAxis(Hand.kRight);

        driverLeftBumper = mController0.getBumper(Hand.kLeft);
        driverRightBumper = mController0.getBumper(Hand.kRight);
        
        driverPOV = mController0.getPOV();

        opLeftStickY = mController1.getX(Hand.kLeft);
        opLeftStickX = mController0.getY(Hand.kLeft);
        opRightStickY = mController0.getX(Hand.kRight);
        opRightStickX = mController0.getY(Hand.kRight);

        opA = mController1.getAButton();
        opB = mController1.getBButton();
        opX = mController1.getXButton();
        opY = mController1.getYButton();

        opLeftBumper = mController1.getBumper(Hand.kLeft);
        opRightBumper = mController1.getBumper(Hand.kRight);

        opRightTrigger = mController1.getTriggerAxis(Hand.kRight);
        opLeftTrigger = mController1.getTriggerAxis(Hand.kLeft);

        opPOV = mController1.getPOV();
    }

    public void dashboard() {
        //unused
    }

}