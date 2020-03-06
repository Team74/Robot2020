package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Shooter;
import frc.robot.Shooter.TurretState;

public class ScoreBallsAction implements Action {

    public final double startTime;
    public final double maxRunTime;
    public final int numberBalls;

    public final Shooter shooter;

    public ScoreBallsAction(double maxRunTime, int numberBalls) {
        shooter = Shooter.getInstance();
        startTime = Timer.getFPGATimestamp();
        this.maxRunTime = maxRunTime;
        this.numberBalls = numberBalls;
    }

    public void start() {

    }

    public void done() {
        shooter.setIsShooterOn(false);
        shooter.setIsInxerOn(false, false);
    }

    public boolean isFinished() {
        //check to see if we've covered the driven distance
        return (maxRunTime <= (Timer.getFPGATimestamp() - startTime));
    }

    public void update() {
        if(shooter.isShooterAligned()) {
            shooter.setIsShooterOn(true);
            if (shooter.isFlywheelUpToSpeed(Constants.kFlywheelSpeed)) {
                shooter.setIsInxerOn(true, false);
            } else {
                shooter.setIsInxerOn(false, false);
            }
        } else {
            shooter.setTurretState(TurretState.Tracking);
        }
    }
}