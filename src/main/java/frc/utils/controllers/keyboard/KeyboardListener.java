package frc.utils.controllers.keyboard;

import com.github.kwhat.jnativehook.GlobalScreen;
import com.github.kwhat.jnativehook.NativeHookException;
import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KeyboardListener implements NativeKeyListener {

    public static void startTrackingKeyboard() {
        waitForTables();
        try {
            GlobalScreen.registerNativeHook();
            GlobalScreen.addNativeKeyListener(new KeyboardListener());
        } catch (NativeHookException e) {
            throw new RuntimeException(e);
        }
    }

    private static void waitForTables() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        instance.startClient4("Keyboard");
        instance.setServer("127.0.0.1");
        instance.startDSClient();

        while (!instance.isConnected()) {
            try {
                Thread.sleep(20);
            } catch (Exception e) {

            }
        }

        System.out.println("CONNECTED");


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
