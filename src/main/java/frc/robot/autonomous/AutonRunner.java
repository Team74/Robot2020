package frc.robot.autonomous;

public class AutonRunner {
    private AutonBase auton;
    private Thread thread = null;

    private AutonRunner() {

    }

    public void setAuton(AutonBase _auton) {
        auton = _auton;
        thread = new Thread(new AutonThread() {
            @Override
            public void run() {
                if (auton != null) {
                    auton.run();
                }
            }
        });
    }

    public void start() {
        if (thread != null) {
            thread.start();
        }
    }

    public void stop() {
        if (auton != null) {
            auton.stop();
        }
        thread = null;
    }

    public AutonBase getAuton() {
        return auton;
    }


}