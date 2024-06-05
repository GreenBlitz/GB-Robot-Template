package frc.utils.laptopcomms;

import frc.utils.controllers.keyboard.KeyboardListener;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;

public class LaptopSender {

    public static final String IP = "";
    public static final int PORT = 1;

    private static PrintWriter sender;


    public static final boolean ENABLE_KEYBOARD = true;

    public static void main(String[] args) {
        laptopLogic();
    }


    private static void startConnection() throws IOException {
        Socket socket = new Socket(IP, PORT);
        sender = new PrintWriter(socket.getOutputStream(), true);
    }

    private static void sendMessage(String message) {
        sender.println(message);
    }

    private static void laptopLogic() {
        laptopStart();
        try {
            startConnection();
            while (true) {
                laptopPeriodic();
                Thread.sleep(20);
            }
        } catch (Exception e) {
            System.out.println("Laptop Periodic Stopped:  " + e);
        }
    }

    private static void laptopStart() {
        sendMessage("TEST");
        if (ENABLE_KEYBOARD) {
            KeyboardListener.startTrackingKeyboard();
        }
    }

    private static void laptopPeriodic() {

    }
}
