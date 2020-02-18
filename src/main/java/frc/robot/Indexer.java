package frc.robot;

public class Indexer implements Updateable {
    
    public static Indexer kInstance;

    public static Indexer getInstance() {
        if (kInstance == null) {
            kInstance = new Indexer();
        }
        return kInstance;
    }

    private IndexerState state;
    private IndexerControlState controlState;

    private Indexer() {
        
    }
    
    public void update(double dt) {

    }

    public void dashboard() {
        
    }

    public void setState(IndexerState newState) {

    }

    public void setControlState(IndexerControlState newState) {
        
    }

    public static enum IndexerState {
        NoBalls,
        Rotate,
        StopRotating,
        UptakeBall,
        StopUptake;
    }

    public static enum IndexerControlState {
        PercentOutput,
        VelocityPID,
        PositionPID,
        MotionMagic;
    }

}