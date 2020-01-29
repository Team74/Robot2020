package frc.robot;

import java.net.*;
import java.io.*;

public class Client implements Runnable {
    private Socket socket = null;
    private DataInputStream response = null;
    private DataOutputStream request = null;

    private boolean running = false;

    private String inputLine = "";
    private String outputLine = "";

    public Client(String _adress, int _port) {
        try {
            socket = new Socket(_adress, _port);
            System.out.println("Connected");
            response = new DataInputStream(new BufferedInputStream(socket.getInputStream()));
            request = new DataOutputStream(socket.getOutputStream());
        } catch(UnknownHostException u) {
            System.out.println(u);
        } catch(IOException io) {
            System.out.println(io);
        }
    }

    @Override
    public void run() {
        running = true;
        while (running) {
            try {
                System.out.printf("Read line and got: %s%n", inputLine);
                request.writeUTF(inputLine);
                request.flush();
                System.out.printf("Requested: %s%n", inputLine);
                outputLine = response.readLine();
                System.out.printf("Recieved: %s%n", outputLine);
            } catch(IOException io) {
                System.out.println(io);
            }
        }

        try {
            response.close();
            request.close();
            socket.close();
        } catch (IOException io) {
            System.out.println(io);
        }
    }

    public void stop() {
        if (running) {
            running = !running;
        }
    }

    public void setRequest(String _request) {
        inputLine = _request;
    }

    public String getCurrentRequest() {
        return inputLine;
    }

    public String getData() {
        return "";
    }
}