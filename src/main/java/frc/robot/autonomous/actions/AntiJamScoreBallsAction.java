package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Shooter;
import frc.robot.Shooter.LimelightLEDState;
import frc.robot.Shooter.TurretState;

public class AntiJamScoreBallsAction implements Action {

    public final double startTime;
    public final double maxRunTime;
    public final int numberBalls;

    public final Shooter shooter;

    public AntiJamScoreBallsAction(double maxRunTime, int numberBalls) {
        shooter = Shooter.getInstance();
        startTime = Timer.getFPGATimestamp();
        this.maxRunTime = maxRunTime;
        this.numberBalls = numberBalls;
    }

    public void start() {
        System.out.println("Start method of score balls action");
        shooter.setIsShooterOn(false);
        shooter.setIsIndexerOn(false, false);
        shooter.setLimelightLEDS(LimelightLEDState.On);
    }

    public void done() {
        System.out.println("Done method of score balls action");
        shooter.setIsShooterOn(false);
        shooter.setIsIndexerOn(false, false);
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
            System.out.println("Shoooter is aligned");
            shooter.setIsShooterOn(true);
            if (shooter.isFlywheelUpToSpeed(Constants.kFlywheelSpeed, false)) {
                System.out.println("Flywheel up to speed");
                if (!shooter.isIndexerJammed()) {
                    shooter.setIsIndexerOn(true, false);
                } else if (shooter.isIndexerJammed()) {
                    shooter.setIsIndexerOn(true, true);
                }
            } else {
                System.out.println("Flywheel getting up to speed");
                shooter.setIsIndexerOn(false, false);
            }
        } else {
            // System.out.println("Aligning turret");
            shooter.setTurretState(TurretState.Tracking);
        }
    }
}