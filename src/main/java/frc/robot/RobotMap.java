package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RobotMap {
    private static RobotMap kInstance = null;

    private CANSparkMax driveLeftFront = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax driveLeftBack = new CANSparkMax(22, CANSparkMaxLowLevel.MotorType.kBrushless);
    public SpeedControllerGroup driveLeft = new SpeedControllerGroup(driveLeftFront, driveLeftBack);

    private CANSparkMax driveRightFront = new CANSparkMax(33, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax driveRightBack = new CANSparkMax(44, CANSparkMaxLowLevel.MotorType.kBrushless);
    public SpeedControllerGroup driveRight = new SpeedControllerGroup(driveRightFront, driveRightBack);

    public DifferentialDrive drive = new DifferentialDrive(driveLeft, driveRight);

    public CANEncoder drive_E_0;
    public CANEncoder drive_E_1;
    public CANEncoder drive_E_2;
    public CANEncoder drive_E_3;

    

    // public AHRS navX = new AHRS(SPI.Port.kMXP, (byte)60);

    private RobotMap() {
        // drive_E_0 = drive_0.getEncoder();
        // drive_E_1 = drive_1.getEncoder();
        // drive_E_2 = drive_2.getEncoder();
        // drive_E_3 = drive_3.getEncoder();
    }

    public static RobotMap getInstance() {
        if (kInstance == null) {
            kInstance = new RobotMap();
        }

        return kInstance;
    }
}