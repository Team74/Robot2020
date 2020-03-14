package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivebase;
import frc.robot.Drivebase.DriveState;

public class DriveTrajectoryAction implements Action {

    private final double startTime;
    private final double maxRunTime;

    private final Drivebase driveBase;

    public DriveTrajectoryAction(double maxRunTime, Object trajectory) {
        driveBase = Drivebase.getInstance();
        startTime = Timer.getFPGATimestamp();
        this.maxRunTime = maxRunTime;
    }

    public void start() {
        System.out.println("Start Method of Drive Trajectory Action");
    }

    public void done() {
        System.out.println("Done Method of Drive Trajectory Action");
    }

    public boolean isFinished() {
        //check to see if we've finished the trajectory or have passed the maximum allowable time.
        return maxRunTime <= (Timer.getFPGATimestamp() - startTime);
    }

    public void update() {
        // System.out.println("Updating drive Trajectory action");
    }
}