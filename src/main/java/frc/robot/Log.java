package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

public class Log {
    public ArrayList<String> buffer = new ArrayList<>();
    //Path of the file
    public String filename = "";

    public Log(String filename) {
        this.filename = filename;
    }

    public void log(String message, Flags flag) {
        double timestamp = Timer.getFPGATimestamp();
        String flagText = "";
        switch (flag) {
            case ERROR:
                flagText = "Error";
                break;
            case DRIVER:
                flagText = "Driver";
                break;
            default:
                flagText = "No Flag";
                break;
        }
        String entry = String.format("%s; %s: %s", timestamp, flagText, message);
        buffer.add(entry);
    }

    public void writeBuffer() {
        try {
            File logFile = new File(filename);
            FileWriter writer = new FileWriter(filename);
        } catch (IOException e) {

        }
    }

    public enum Flags {
        ERROR,
        DRIVER,
        awdj;
    }
}
