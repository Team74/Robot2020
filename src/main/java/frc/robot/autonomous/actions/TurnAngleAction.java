package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivebase;
import frc.robot.Drivebase.DriveState;

public class TurnAngleAction implements Action {
    
    public final Drivebase driveBase;

    public final double startTime;
    public final double maxRunTime;
    public final double turnAngleDeg;

    public TurnAngleAction(double maxRunTime, double turnAngleDeg) {
        driveBase = Drivebase.getInstance();
        startTime = Timer.getFPGATimestamp();
        this.maxRunTime = maxRunTime;
        this.turnAngleDeg = turnAngleDeg;
    }
    
    public void start() {
        driveBase.setDriveState(DriveState.TurnAngle);
        driveBase.setAtTarget(false);
        driveBase.setTargetHeading(this.turnAngleDeg);
    }

    public void done() {

    }

    public boolean isFinished() {
        boolean isFinished = (maxRunTime <= (Timer.getFPGATimestamp() - startTime) || driveBase.atTarget());
        // System.out.println("Checking is turn angle action finished" );
        return isFinished;
    }

    public void update() {
        // System.out.println("Updating turn angle action");
    }
}