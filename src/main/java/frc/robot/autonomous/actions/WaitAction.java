package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.Timer;

public class WaitAction implements Action {

    public double startTime;
    private final double runtime;//This is in seconds

    public WaitAction(double _runTime) {
        runtime = _runTime;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void done() {
        System.out.println("Completed Test Action");
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) >= runtime;
    }

    @Override
    public void update() {

    }
}