package frc.robot;

/*
Class that handles *human* input; for sensors, see SensorManager. Since the various TeleopMaster subclasses are the only thing that should be touching this class, it can get updated from there.
*/

import frc.robot.Updateable;

//import frc.lib.utils.Utilities;

import java.util.HashMap;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;


public class InputManager implements Updateable {
    public static InputManager kInstance = null; 

    private XboxController mController0 = new XboxController(0);
    private XboxController mController1 = new XboxController(1);

    //private double kDeadband = 0.05;

    //All these are values that can be read from outside the class
    // public HashMap<String, Boolean> driverButtons  = new HashMap<String, Boolean>();
    // public HashMap<String, Double> driverJoysticks  = new HashMap<String, Double>();

    public double driverLeftY = 0;
    public double driverLeftX = 0;
    public double driverRightY = 0;
    public double driverRightX = 0;

    public static InputManager getInstance() {
        if (kInstance == null) {
            kInstance = new InputManager();
        }
        return kInstance;
    }

    private InputManager() {
        
    }

    public void update(double dt) {
        driverLeftY = mController0.getY(Hand.kLeft);
        driverLeftX = mController0.getX(Hand.kLeft);
        driverRightY = mController0.getY(Hand.kRight);
        driverRightX = mController0.getX(Hand.kRight);
    }

}