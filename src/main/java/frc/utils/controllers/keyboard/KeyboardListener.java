package frc.utils.controllers.keyboard;

import com.github.kwhat.jnativehook.GlobalScreen;
import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;

public class KeyboardListener implements NativeKeyListener {

    public static final String IP = "";
    public static final int PORT = 1;

    private static PrintWriter sender;

    public static void startTrackingKeyboard() {
        try {
            startConnection();
            GlobalScreen.registerNativeHook();
            GlobalScreen.addNativeKeyListener(new KeyboardListener());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    private static void startConnection() throws IOException {
        Socket socket = new Socket(IP, PORT);
        sender = new PrintWriter(socket.getOutputStream(), true);
    }

    private static void sendMessage(String message) {
        sender.println(message);
    }

    @Override
    public void nativeKeyPressed(NativeKeyEvent nativeEvent) {
        SmartDashboard.putBoolean(getName(nativeEvent), true);
    }

    public String getName(NativeKeyEvent event) {
        return "Keyboard/" + NativeKeyEvent.getKeyText(event.getKeyCode());
    }

    @Override
    public void nativeKeyReleased(NativeKeyEvent nativeEvent) {
        SmartDashboard.putBoolean(getName(nativeEvent), false);
    }
}
