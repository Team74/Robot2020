package frc.robot;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class Climber {
    private InputManager inputManager;
    private BaseMotorController climbMotor;
    private BaseMotorController balenceMotor;

    private int climberState = 0;
    private int balenceState = 0;

    public Climber(InputManager inputManager, RobotMap robotMap) {
        climbMotor = robotMap.climber;
        balenceMotor = robotMap.cliberBalence;
        this.inputManager = inputManager;
    }

    public void handleInput() {
        if (inputManager.driverPOV != -1) {
            if(inputManager.driverPOV == 0) {
                climberState = -1;
            } else if (inputManager.driverPOV == 90) {
                balenceState = -1;
            } else if (inputManager.driverPOV == 180) {
                climberState = 1;
            } else if (inputManager.driverPOV == 270) {
                balenceState = 1;
            }
        } else {
            balenceState = 0;
            climberState = 0;
        } 
    }

    public void update() {
        switch(climberState) {
            case 1:
            System.out.println("Retract");
                break;
            case -1:
                System.out.println("Extend");
                break;
            case 0:
                System.out.println("Set to 0");
                break;
        }

        switch(balenceState) {
            case 1:
                System.out.println("Left");
                break;
            case -1:
            System.out.println("Right");
                break;
            case 0:
            System.out.println("Set to 0");
        }
    }
}