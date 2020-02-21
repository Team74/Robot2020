package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.autonomous.actions.Action;

public abstract class AutonBase {
    private double updateRate = 1.0 / 50.0;

    private boolean isActive = false;

    protected abstract void routine();

    public void run() {
        isActive = true;
        routine();
        done();
    }

    public void done() {
        System.out.println("Auton Done");
    }

    public void stop() {
        isActive = false;
    }

    public boolean isActive() {
        return isActive;
    }

    public void runAction(Action action) {
        action.start();

        while (isActive() && !(action.isFinished())) {
            action.update();

            long waitTime = (long) updateRate * 1000;
            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        action.done();
        System.out.println("Action completed in: " + (Timer.getFPGATimestamp() - action.startTime));

    }
    
}