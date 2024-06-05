package frc.utils.controllers.keyboard;

import com.github.kwhat.jnativehook.GlobalScreen;
import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;
import frc.utils.laptopcomms.LaptopSender;

import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

public class KeyboardListener implements NativeKeyListener {


    private static PrintWriter sender;

    public static void main(String[] args) {
        startTrackingKeyboard();
    }

    private static void startConnection() {
        try {
            new ServerSocket(LaptopSender.PORT);
            System.out.println("SERVER CREATED");
            Socket socket = new Socket(LaptopSender.ROBOT_IP, LaptopSender.PORT);
            System.out.println("SERVER ACCEPTED");
            sender = new PrintWriter(socket.getOutputStream(), true);
            System.out.println("SENDER CREATED");
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    public static void sendMessage(String message) {
        try {
            sender.println(message);
            System.out.println("Sent Message: " + message);
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    public static void startTrackingKeyboard() {
        startConnection();
        try {
            GlobalScreen.registerNativeHook();
            GlobalScreen.addNativeKeyListener(new KeyboardListener());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void nativeKeyPressed(NativeKeyEvent nativeEvent) {
        sendMessage(getName(nativeEvent) + ", true");
    }

    public String getName(NativeKeyEvent event) {
        return "Keyboard/" + NativeKeyEvent.getKeyText(event.getKeyCode());
    }

    @Override
    public void nativeKeyReleased(NativeKeyEvent nativeEvent) {
        sendMessage(getName(nativeEvent) + ", false");
    }
}
