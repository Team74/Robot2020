package frc.robot;

import java.util.HashMap;

/**
 * This will 
 */
public class Vision {
    private Thread thread = null;
    private String adress = "chaoticpi.local";
    private int port = 9999;
    private Client client = new Client(adress, port);

    private static Vision kInstance = null;
    public static Vision getInstance() {
        if (kInstance == null) {
            kInstance = new Vision();
        }
        return kInstance;
    }

    private Vision() {
        thread = new Thread(client);
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

    public HashMap<String, Integer> getData() {
        HashMap<String, Integer> processedData = new HashMap<String, Integer>();
        String data = client.getData();
        String[] pairs = data.split(";");
        for(String i : pairs) {
            String[] pair = i.split(":");
            processedData.put(pair[0], Integer.parseInt(pair[1]));
        }
        return processedData;
    }

    public void setInput(String _input) {
        client.setInput(_input);
    }
}