package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import com.revrobotics.CANDigitalInput.LimitSwitch;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RobotMap {
    private static RobotMap kInstance = null;

     public CANSparkMax driveLeftFront = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
     public CANSparkMax driveLeftBack = new CANSparkMax(22, CANSparkMaxLowLevel.MotorType.kBrushless);
     public SpeedControllerGroup driveLeft = new SpeedControllerGroup(driveLeftFront, driveLeftBack);

     public CANSparkMax driveRightFront = new CANSparkMax(44, CANSparkMaxLowLevel.MotorType.kBrushless);
     public CANSparkMax driveRightBack = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
     public SpeedControllerGroup driveRight = new SpeedControllerGroup(driveRightFront, driveRightBack);

     public DifferentialDrive drive = new DifferentialDrive(driveLeft, driveRight);

    // public CANEncoder drive_E_0;
    // public CANEncoder drive_E_1;
    // public CANEncoder drive_E_2;
    // public CANEncoder drive_E_3;

    public TalonSRX intake = new TalonSRX(18);         
    public VictorSPX flywheel = new VictorSPX(6);
    public VictorSPX test = new VictorSPX(6);
    public TalonSRX turret = new TalonSRX(0);
    public TalonSRX hood = new TalonSRX(0);  
    public TalonSRX indexer = new TalonSRX(1);
    public VictorSPX uptake = new VictorSPX(7); 
    

    // public TalonSRX climber = new TalonSRX(0);
    // public TalonSRX cliberBalence= new TalonSRX(0);
    //find CAN id

    public DigitalInput uptakeLimit = new DigitalInput(0);
    public DigitalInput indexerRotationLimit = new DigitalInput(1);
    public DigitalInput [] ballLimit = {new DigitalInput(2), 
                                        new DigitalInput(3), 
                                        new DigitalInput(4), 
                                        new DigitalInput(5), 
                                        new DigitalInput(6)};

    public Compressor compressor = new Compressor(0);
    public DoubleSolenoid gearShift = new DoubleSolenoid(49, 0, 1);
    public DoubleSolenoid intakeArm = new DoubleSolenoid(49, 2, 3);


     //public AHRS navX = new AHRS(SPI.Port.kMXP, (byte)60);

    private RobotMap() {
        // drive_E_0 = drive_0.getEncoder();
        // drive_E_1 = drive_1.getEncoder();
        // drive_E_2 = drive_2.getEncoder();
        // drive_E_3 = drive_3.getEncoder();
        //flywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        driveLeft.setInverted(true);
        driveRight.setInverted(true);
    }

    public static RobotMap getInstance() {
        if (kInstance == null) {
            kInstance = new RobotMap();
        }

        return kInstance;
    }
}