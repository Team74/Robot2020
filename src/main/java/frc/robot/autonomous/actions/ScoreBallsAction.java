package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Shooter;

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

    }

    public boolean isFinished() {
        //check to see if we've covered the driven distance
        return true;
    }

    public void update() {
    }
}