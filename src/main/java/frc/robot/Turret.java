package frc.robot;

public class Turret implements Updateable {
    
    public static Turret kInstance;

    public static Turret getInstance() {
        if (kInstance == null) {
            kInstance = new Turret();
        }
        return kInstance;
    }

    private TurretState state;
    private TurretControlState controlState;

    private Turret() {

    }
    
    public void update(double dt) {

    }

    public void dashboard() {
        
    }

    public void setState(IndexerState newState) {

    }

    public void setControlState(IndexerControlState newState) {
        
    }

    public static enum TurretState {
        Tracking,
        Holding,
        ReadyToShoot,
        Manual;
    }

    public static enum TurretControlState {
        PercentOutput,
        VelocityPID,
        PositionPID,
        MotionMagic;
    }

}