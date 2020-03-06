package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivebase;
import frc.robot.Drivebase.DriveState;

public class StopAction implements Action {

    public final double startTime;
    public final double maxRunTime;

    public final Drivebase driveBase;

    public StopAction(double maxRunTime) {
        driveBase = Drivebase.getInstance();
        startTime = Timer.getFPGATimestamp();
        this.maxRunTime = maxRunTime;
    }

    public void start() {
        driveBase.setDriveState(DriveState.Teleop);
    }

    public void done() {

    }

    public boolean isFinished() {
        //check to see if we've covered the driven distance
        return true;
    }

    public void update() {
        System.out.println("Updating stop action");
    }
}