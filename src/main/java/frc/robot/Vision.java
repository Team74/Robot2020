package frc.robot;

/**
 * This will 
 */
public class Vision {
    private Thread thread = null;
    private String adress = "chaoticpi.local";
    private int port = 9999;

    private static Vision kInstance = null;
    public static Vision getInstance() {
        if (kInstance == null) {
            kInstance = new Vision();
        }
        return kInstance;
    }

    private Vision() {
        thread = new Thread(new Client(adress, port));
    }

    public void start() {
        if (thread != null) {
            thread.start();
        }
    }

    public void stop() {
        if (thread != null) {
            thread.stop();
        }
        thread = null;
        kInstance = null;
    }
}