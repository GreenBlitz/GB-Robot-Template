package frc.utils.controllers.keyboard;

import com.github.kwhat.jnativehook.GlobalScreen;
import com.github.kwhat.jnativehook.NativeHookException;
import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;


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
        System.out.println(NativeKeyEvent.getKeyText(nativeEvent.getKeyCode()));
    }

    @Override
    public void nativeKeyReleased(NativeKeyEvent nativeEvent) {

    }
}
