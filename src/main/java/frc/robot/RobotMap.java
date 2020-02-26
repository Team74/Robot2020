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
    private final int kTimeoutMs = 30;

    private static RobotMap kInstance = null;

    public CANSparkMax leftDriveMaster  = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    public CANSparkMax leftDriveFollower = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    public CANSparkMax rightDriveMaster = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
    public CANSparkMax rightDriveFollower = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);

    public VictorSPX intake = new VictorSPX(7);         
    public TalonSRX flywheel = new TalonSRX(17);
    public TalonSRX turret = new TalonSRX(18);
    public TalonSRX hood = new TalonSRX(20);  
    public TalonSRX indexer = new TalonSRX(21);
    public VictorSPX uptake = new VictorSPX(23); 
    

    // public TalonSRX climber = new TalonSRX(0);
    // public TalonSRX cliberBalence= new TalonSRX(0);
    //find CAN id

    public DigitalInput hoodLimit = new DigitalInput(0);
    public DigitalInput turretLimit = new DigitalInput(1);
    public DigitalInput indexerRotationLimit = new DigitalInput(2);
    public DigitalInput [] ballLimit = {new DigitalInput(3), 
                                        new DigitalInput(4), 
                                        new DigitalInput(5), 
                                        new DigitalInput(6), 
                                        new DigitalInput(7)};

    // public Compressor compressor = new Compressor(0);
    // public DoubleSolenoid gearShift = new DoubleSolenoid(49, 0, 1);
    // public DoubleSolenoid intakeArm = new DoubleSolenoid(49, 2, 3);

    //Limelight network table
    public NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public AHRS navX = new AHRS(SPI.Port.kMXP, (byte)60);

    private RobotMap() {
        flywheel.configFactoryDefault(kTimeoutMs);
        flywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
        flywheel.setSensorPhase(true);
        flywheel.setInverted(false);
        flywheel.configNominalOutputForward(0);
        flywheel.configNominalOutputReverse(0);
        flywheel.configPeakOutputForward(1.00);
        flywheel.configPeakOutputReverse(0);

        flywheel.config_kF(0, Constants.kFlywheelF, kTimeoutMs);
        flywheel.config_kP(0, Constants.kFlywheelP, kTimeoutMs);
        flywheel.config_kI(0, Constants.kFlywheelI, kTimeoutMs);
        flywheel.config_kD(0, Constants.kFlywheelD, kTimeoutMs);

        hood.configFactoryDefault(kTimeoutMs);
        hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
        hood.setSensorPhase(true);
        hood.setInverted(true);
        hood.configNominalOutputForward(0);
        hood.configNominalOutputReverse(0);
        hood.configPeakOutputForward(.25);
        hood.configPeakOutputReverse(-.25);
        hood.selectProfileSlot(0, 0);
		hood.configMotionCruiseVelocity(Constants.kHoodMaxVelocity, kTimeoutMs);
		hood.configMotionAcceleration(Constants.kHoodMaxAcceleration, kTimeoutMs);
        hood.config_kF(0, Constants.kHoodF, kTimeoutMs);
        hood.config_kP(0, Constants.kHoodP, kTimeoutMs);
        hood.config_kI(0, Constants.kHoodI, kTimeoutMs);
        hood.config_kD(0, Constants.kHoodD, kTimeoutMs);

        turret.configFactoryDefault(kTimeoutMs);
        turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
        int pulseWidth = turret.getSensorCollection().getPulseWidthPosition();
        pulseWidth = pulseWidth & 0xFFF;
        turret.getSensorCollection().setQuadraturePosition(pulseWidth, kTimeoutMs);
        turret.setSensorPhase(false);
        turret.setInverted(true);
        turret.configNominalOutputForward(0);
        turret.configNominalOutputReverse(0);
        turret.configPeakOutputForward(1.00);
        turret.configPeakOutputReverse(-1.00);

        turret.config_kF(0, Constants.kTurretF, kTimeoutMs);
        turret.config_kP(0, Constants.kTurretP, kTimeoutMs);
        turret.config_kI(0, Constants.kTurretI, kTimeoutMs);
        turret.config_kD(0, Constants.kTurretD, kTimeoutMs);

        uptake.setInverted(true);
    }

    public static RobotMap getInstance() {
        if (kInstance == null) {
            kInstance = new RobotMap();
        }

        return kInstance;
    }
}