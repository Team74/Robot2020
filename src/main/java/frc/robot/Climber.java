package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class Climber implements Updateable {
    private InputManager inputManager;
    private BaseMotorController climbMotor;
    private BaseMotorController balenceMotor;

    private int climberState = 0;
    private int balenceState = 0;

    public Climber(InputManager inputManager, RobotMap robotMap) {
        // climbMotor = robotMap.climber;
        // balenceMotor = robotMap.cliberBalence;
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

    public void update(double dt) {
        switch(climberState) {
            case 1:
                climbMotor.set(ControlMode.PercentOutput, -50);
                break;
            case -1:
                climbMotor.set(ControlMode.PercentOutput, 50);
                break;
            case 0:
               climbMotor.set(ControlMode.PercentOutput, 0);
                break;
        }

        switch(balenceState) {
            case 1:
                balenceMotor.set(ControlMode.PercentOutput, 50);
                break;
            case -1:
                balenceMotor.set(ControlMode.PercentOutput, -50);
                break;
            case 0:
                balenceMotor.set(ControlMode.PercentOutput, 0);
        }
    }
}