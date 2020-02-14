package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RobotMap {
    private static RobotMap kInstance = null;

     public CANSparkMax driveLeftFront = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
     public CANSparkMax driveLeftBack = new CANSparkMax(22, CANSparkMaxLowLevel.MotorType.kBrushless);
     public SpeedControllerGroup driveLeft = new SpeedControllerGroup(driveLeftFront, driveLeftBack);

     public CANSparkMax driveRightFront = new CANSparkMax(44, CANSparkMaxLowLevel.MotorType.kBrushless);
     public CANSparkMax driveRightBack = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
     public SpeedControllerGroup driveRight = new SpeedControllerGroup(driveRightFront, driveRightBack);

    public CANEncoder driveLeftFrontEncoder;
    public CANEncoder driveLeftBackEncoder;
    public CANEncoder driveRightFrontEncoder;
    public CANEncoder driveRightBackEncoder;

    public TalonSRX intake = new TalonSRX(0);         
    public VictorSPX flywheel = new VictorSPX(6);
    public VictorSPX test = new VictorSPX(6);
    public TalonSRX turret = new TalonSRX(18);
    public TalonSRX hood = new TalonSRX(0);  
    public TalonSRX indexer = new TalonSRX(1);
    public VictorSPX uptake = new VictorSPX(7); 
    

    public TalonSRX climber = new TalonSRX(0);
    public TalonSRX cliberBalence= new TalonSRX(0);
    //find CAN id

    public DigitalInput uptakeLimit = new DigitalInput(0);
    public DigitalInput indexerRotationLimit = new DigitalInput(1);
    public DigitalInput [] ballLimit = {new DigitalInput(2), 
                                        new DigitalInput(3), 
                                        new DigitalInput(4), 
                                        new DigitalInput(5), 
                                        new DigitalInput(6)};

    // public Compressor compressor = new Compressor(0);
    public DoubleSolenoid gearShift = new DoubleSolenoid(49, 0, 1);
    public DoubleSolenoid intakeArm = new DoubleSolenoid(49, 2, 3);

    //Limelight network table
    public NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

     public AHRS navX = new AHRS(SPI.Port.kMXP, (byte)60);

    private RobotMap() {
        driveLeftFrontEncoder = driveLeftFront.getEncoder();
        driveLeftBackEncoder = driveLeftBack.getEncoder();
        driveRightFrontEncoder = driveRightFront.getEncoder();
        driveRightBackEncoder = driveRightBack.getEncoder();
        flywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

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