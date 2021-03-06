package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivebase;
import frc.robot.Drivebase.DriveState;

public class DriveDistanceAction implements Action {

    private final double startTime;
    private final double maxRunTime;
    private final double targetDistanceIn;

    private final Drivebase driveBase;

    public DriveDistanceAction(double maxRunTime, double targetDistanceIn) {
        driveBase = Drivebase.getInstance();
        startTime = Timer.getFPGATimestamp();
        this.maxRunTime = maxRunTime;
        this.targetDistanceIn = targetDistanceIn;
    }

    public void start() {
        System.out.println("Start Method of Drive Distance Action");
        driveBase.setDriveState(DriveState.DriveStraight);
        driveBase.setAtTarget(false);
        driveBase.setTargetDistance(this.targetDistanceIn);
        driveBase.zeroDriveEncoders();
    }

    public void done() {
        System.out.println("Done Method of Drive Distance Action");
    }

    public boolean isFinished() {
        //check to see if we've covered the driven distance
        return (driveBase.atTarget() || (maxRunTime <= (Timer.getFPGATimestamp() - startTime)));
    }

    public void update() {
        // System.out.println("Updating drive distance action");
    }
}