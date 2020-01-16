package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.lib.motorcontroller.WrappedSparkMax;

public class RobotMap {

    public static RobotMap instance;

    public static RobotMap getInstance() {
        if (instance == null) {
            instance = new RobotMap();
        }
        return instance;
    }

    public final WrappedSparkMax leftMaster;
    private final WrappedSparkMax leftSlave;

    public final WrappedSparkMax rightMaster;
    private final WrappedSparkMax rightSlave;

    public RobotMap() {
        leftMaster = new WrappedSparkMax(1);
        leftSlave = WrappedSparkMax.createSlave(2, MotorType.kBrushless, leftMaster);

        rightMaster = new WrappedSparkMax(3);
        rightSlave = WrappedSparkMax.createSlave(4, MotorType.kBrushless, rightMaster);
    }
}