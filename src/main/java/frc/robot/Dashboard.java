package frc.robot;

public class Dashboard {

    public static Dashboard kInstance;

    public static Dashboard getInstance() {
        if (kInstance == null) {
            kInstance = new Dashboard();
        }
        return kInstance;
    }
}