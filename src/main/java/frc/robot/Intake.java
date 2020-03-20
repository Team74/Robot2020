package frc.robot;

public class Intake implements Updateable {
    
    public static Intake kInstance;

    public static Intake getInstance() {
        if (kInstance == null) {
            kInstance = new Intake();
        }
        return kInstance;
    }

    private IntakeState state;
    private IntakeControlState controlState;

    private Intake() {
        
    }
    
    public void update(double dt) {

    }

    public void dashboard() {
        
    }

    public void setState(IntakeState newState) {

    }

    public void setControlState(IntakeControlState newState) {
        
    }

    public static enum IntakeState {
        IntakeUp,
        IntakeDown,
        IntakeDownRev,
        Indexing; //We dont know if this will be needed, not currently called
    }

    public static enum IntakeControlState {
        PercentOutput,
        VelocityPID,
        PositionPID,
        MotionMagic;
    }

}