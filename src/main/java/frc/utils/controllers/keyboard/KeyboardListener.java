package frc.utils.controllers.keyboard;

import com.github.kwhat.jnativehook.GlobalScreen;
import com.github.kwhat.jnativehook.NativeHookException;
import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KeyboardListener implements NativeKeyListener {

    public static void startTrackingKeyboard() {
        try {
            GlobalScreen.registerNativeHook();
            GlobalScreen.addNativeKeyListener(new KeyboardListener());
        } catch (NativeHookException e) {
            throw new RuntimeException(e);
        }
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
