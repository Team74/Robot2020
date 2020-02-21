package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivebase;
import frc.robot.Drivebase.DriveState;

public class DriveDistanceAction implements Action {

    public final double startTime;
    public final double maxRunTime;
    public final double driveDistanceIn;

    public final Drivebase driveBase;

    public DriveDistanceAction(double maxRunTime, double driveDistanceIn) {
        driveBase = Drivebase.getInstance();
        startTime = Timer.getFPGATimestamp();
        this.maxRunTime = maxRunTime;
        this.driveDistanceIn = driveDistanceIn;
    }

    public void start() {
        driveBase.setDriveState(DriveState.Autonomous);
    }

    public void done() {

    }

    public boolean isFinished() {
        //check to see if we've covered the driven distance
    }

    public void update() {
        if (!isFinished() || !(maxRunTime <= (Timer.getFPGATimestamp() - startTime))) {
            driveBase.driveDistance(this.driveDistanceIn);
        } else {       
            done();
        }
    }
}