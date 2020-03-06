package frc.robot.autonomous;

public class AutonRunner {
    private static AutonRunner kInstance;

    private AutonBase auton;
    private Thread thread = null;

    private AutonRunner() {

    }

    public static AutonRunner getInstance() {
        if (kInstance == null) {
            kInstance = new AutonRunner();
        }
        return kInstance;
    }

    public void setAuton(AutonBase _auton) {
        System.out.println("Setting auto mode");
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
        System.out.println("Starting auto mode in auto runner");
        if (thread != null) {
            thread.start();
        }
    }

    public void stop() {
        System.out.println("Stopping auto mode in auto runner");
        if (auton != null) {
            auton.stop();
        }
        thread = null;
    }

    public AutonBase getAuton() {
        return auton;
    }


}