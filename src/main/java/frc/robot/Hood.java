package frc.robot;

public class Hood implements Updateable {
    
    public static Hood kInstance;

    public static Hood getInstance() {
        if (kInstance == null) {
            kInstance = new Hood();
        }
        return kInstance;
    }

    private HoodState state;
    private HoodControlState controlState;

    private Hood() {
        
    }
    
    public void update(double dt) {

    }

    public void dashboard() {
        
    }

    public void setState(IndexerState newState) {

    }

    public void setControlState(IndexerControlState newState) {
        
    }

    public static enum HoodState {
        Raising,
        Lowering,
        Holding,
        Automatic,
        Zeroing;
    }

    public static enum HoodControlState {
        PercentOutput,
        VelocityPID,
        PositionPID,
        MotionMagic;
    }

}