package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivebase;
import frc.robot.Drivebase.DriveState;

public class DriveDistanceAction implements Action {

    public final double startTime;
    public final double maxRunTime;
    public final double targetDistanceIn;

    public final Drivebase driveBase;

    public DriveDistanceAction(double maxRunTime, double targetDistanceIn) {
        driveBase = Drivebase.getInstance();
        startTime = Timer.getFPGATimestamp();
        this.maxRunTime = maxRunTime;
        this.targetDistanceIn = targetDistanceIn;
    }

    public void start() {
        driveBase.setDriveState(DriveState.DriveStraight);
        driveBase.setAtTarget(false);
        driveBase.setTargetDistance(this.targetDistanceIn);
        driveBase.zeroDriveEncoders();
    }

    public void done() {

    }

    public boolean isFinished() {
        //check to see if we've covered the driven distance
        return (driveBase.atTarget() || !(maxRunTime <= (Timer.getFPGATimestamp() - startTime)));
    }

    public void update() {
    }
}