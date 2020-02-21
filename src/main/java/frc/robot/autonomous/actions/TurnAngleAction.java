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
        driveBase.setDriveState(DriveState.Autonomous);
    }

    public void done() {

    }

    public boolean isFinished() {
        //Check to see if we turned to the desired angle
    }

    public void update() {
        if (!isFinished() || !(maxRunTime <= (Timer.getFPGATimestamp() - startTime))) {
            driveBase.turnToAngle(this.turnAngleDeg);
        } else {       
            done();
        }
    }
}