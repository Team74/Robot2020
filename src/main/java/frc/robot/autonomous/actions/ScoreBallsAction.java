package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Shooter;
import frc.robot.Shooter.LimelightLEDState;
import frc.robot.Shooter.TurretState;

public class ScoreBallsAction implements Action {

    private final double startTime;
    private final double maxRunTime;
    private final int numberBalls;

    private final Shooter shooter;

    public ScoreBallsAction(double maxRunTime, int numberBalls) {
        shooter = Shooter.getInstance();
        startTime = Timer.getFPGATimestamp();
        this.maxRunTime = maxRunTime;
        this.numberBalls = numberBalls;
    }

    public void start() {
        System.out.println("Start method of score balls action");
        shooter.setIsShooterOn(false);
        // shooter.setIsIndexerOn(false, false);
        shooter.setLimelightLEDS(LimelightLEDState.On);
    }

    public void done() {
        System.out.println("Done method of score balls action");
        shooter.setIsShooterOn(false);
        // shooter.setIsIndexerOn(false, false);
        shooter.setLimelightLEDS(LimelightLEDState.Off);
        shooter.setTurretState(TurretState.Holding);
    }

    public boolean isFinished() {
        //check elapsed time
        double timeDiff = Timer.getFPGATimestamp() - startTime;
        return (maxRunTime <= timeDiff);
    }

    public void update() {
        shooter.setLimelightLEDS(LimelightLEDState.On);
        if(shooter.isShooterAligned()) {
            shooter.setIsShooterOn(true);
            // if (shooter.isFlywheelUpToSpeed(Constants.kFlywheelSpeed, false)) {
            //     shooter.setIsIndexerOn(true, false);
            // } else {
            //     shooter.setIsIndexerOn(false, false);
            // }
        } else {
            // System.out.println("Aligning turret");
            shooter.setTurretState(TurretState.Tracking);
        }
    }
}