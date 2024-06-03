package frc.utils.controllers.keyboard;

import com.github.kwhat.jnativehook.GlobalScreen;
import com.github.kwhat.jnativehook.NativeHookException;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.utils.controllers.Controller;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class KeyboardController implements Controller{

    public final Trigger
            ESC, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10,
            F11, F12, DELETE, BACKTICK, ONE, TWO, THREE, FOUR,
            FIVE, SIX, SEVEN, EIGHT, NINE, ZERO, MINUS, EQUALS,
            BACKSPACE, TAB, Q, W, E, R, T, Y, U, I, O, P, A, S,
            D, F, G, H, J, K, L, SEMICOLON, APOSTROPHE, LEFT_SHIFT,
            Z, X, C, V, B, N, M, COMMA, PERIOD,
            RIGHT_SHIFT, LEFT_CONTROL, LEFT_ALT, RIGHT_CONTROL,
            LEFT_ARROW, RIGHT_ARROW, UP_ARROW, DOWN_ARROW, NUMPAD_0, NUMPAD_1, NUMPAD_2,
            NUMPAD_3, NUMPAD_4, NUMPAD_5, NUMPAD_6, NUMPAD_7, NUMPAD_8,
            NUMPAD_9;

    /**
     * Construct an instance of a device.
     */
    public KeyboardController() {
        ESC = new Trigger(new LoggedDashboardBoolean("keyboard/esc", false)::get);
        F1 = new Trigger(new LoggedDashboardBoolean("keyboard/f1", false)::get);
        F2 = new Trigger(new LoggedDashboardBoolean("keyboard/f2", false)::get);
        F3 = new Trigger(new LoggedDashboardBoolean("keyboard/f3", false)::get);
        F4 = new Trigger(new LoggedDashboardBoolean("keyboard/f4", false)::get);
        F5 = new Trigger(new LoggedDashboardBoolean("keyboard/f5", false)::get);
        F6 = new Trigger(new LoggedDashboardBoolean("keyboard/f6", false)::get);
        F7 = new Trigger(new LoggedDashboardBoolean("keyboard/f7", false)::get);
        F8 = new Trigger(new LoggedDashboardBoolean("keyboard/f8", false)::get);
        F9 = new Trigger(new LoggedDashboardBoolean("keyboard/f9", false)::get);
        F10 = new Trigger(new LoggedDashboardBoolean("keyboard/f10", false)::get);
        F11 = new Trigger(new LoggedDashboardBoolean("keyboard/f11", false)::get);
        F12 = new Trigger(new LoggedDashboardBoolean("keyboard/f12", false)::get);
        DELETE = new Trigger(new LoggedDashboardBoolean("keyboard/delete", false)::get);
        BACKTICK = new Trigger(new LoggedDashboardBoolean("keyboard/`", false)::get);
        ONE = new Trigger(new LoggedDashboardBoolean("keyboard/1", false)::get);
        TWO = new Trigger(new LoggedDashboardBoolean("keyboard/2", false)::get);
        THREE = new Trigger(new LoggedDashboardBoolean("keyboard/3", false)::get);
        FOUR = new Trigger(new LoggedDashboardBoolean("keyboard/4", false)::get);
        FIVE = new Trigger(new LoggedDashboardBoolean("keyboard/5", false)::get);
        SIX = new Trigger(new LoggedDashboardBoolean("keyboard/6", false)::get);
        SEVEN = new Trigger(new LoggedDashboardBoolean("keyboard/7", false)::get);
        EIGHT = new Trigger(new LoggedDashboardBoolean("keyboard/8", false)::get);
        NINE = new Trigger(new LoggedDashboardBoolean("keyboard/9", false)::get);
        ZERO = new Trigger(new LoggedDashboardBoolean("keyboard/0", false)::get);
        MINUS = new Trigger(new LoggedDashboardBoolean("keyboard/-", false)::get);
        EQUALS = new Trigger(new LoggedDashboardBoolean("keyboard/=", false)::get);
        BACKSPACE = new Trigger(new LoggedDashboardBoolean("keyboard/backspace", false)::get);
        TAB = new Trigger(new LoggedDashboardBoolean("keyboard/tab", false)::get);
        Q = new Trigger(new LoggedDashboardBoolean("keyboard/q", false)::get);
        W = new Trigger(new LoggedDashboardBoolean("keyboard/w", false)::get);
        E = new Trigger(new LoggedDashboardBoolean("keyboard/e", false)::get);
        R = new Trigger(new LoggedDashboardBoolean("keyboard/r", false)::get);
        T = new Trigger(new LoggedDashboardBoolean("keyboard/t", false)::get);
        Y = new Trigger(new LoggedDashboardBoolean("keyboard/y", false)::get);
        U = new Trigger(new LoggedDashboardBoolean("keyboard/u", false)::get);
        I = new Trigger(new LoggedDashboardBoolean("keyboard/i", false)::get);
        O = new Trigger(new LoggedDashboardBoolean("keyboard/o", false)::get);
        P = new Trigger(new LoggedDashboardBoolean("keyboard/p", false)::get);
        A = new Trigger(new LoggedDashboardBoolean("keyboard/a", false)::get);
        S = new Trigger(new LoggedDashboardBoolean("keyboard/s", false)::get);
        D = new Trigger(new LoggedDashboardBoolean("keyboard/d", false)::get);
        F = new Trigger(new LoggedDashboardBoolean("keyboard/f", false)::get);
        G = new Trigger(new LoggedDashboardBoolean("keyboard/g", false)::get);
        H = new Trigger(new LoggedDashboardBoolean("keyboard/h", false)::get);
        J = new Trigger(new LoggedDashboardBoolean("keyboard/j", false)::get);
        K = new Trigger(new LoggedDashboardBoolean("keyboard/k", false)::get);
        L = new Trigger(new LoggedDashboardBoolean("keyboard/l", false)::get);
        SEMICOLON = new Trigger(new LoggedDashboardBoolean("keyboard/;", false)::get);
        APOSTROPHE = new Trigger(new LoggedDashboardBoolean("keyboard/'", false)::get);
        LEFT_SHIFT = new Trigger(new LoggedDashboardBoolean("keyboard/shift", false)::get);
        Z = new Trigger(new LoggedDashboardBoolean("keyboard/z", false)::get);
        X = new Trigger(new LoggedDashboardBoolean("keyboard/x", false)::get);
        C = new Trigger(new LoggedDashboardBoolean("keyboard/c", false)::get);
        V = new Trigger(new LoggedDashboardBoolean("keyboard/v", false)::get);
        B = new Trigger(new LoggedDashboardBoolean("keyboard/b", false)::get);
        N = new Trigger(new LoggedDashboardBoolean("keyboard/n", false)::get);
        M = new Trigger(new LoggedDashboardBoolean("keyboard/m", false)::get);
        COMMA = new Trigger(new LoggedDashboardBoolean("keyboard/,", false)::get);
        PERIOD = new Trigger(new LoggedDashboardBoolean("keyboard/.", false)::get);
        RIGHT_SHIFT = new Trigger(new LoggedDashboardBoolean("keyboard/right shift", false)::get);
        LEFT_CONTROL = new Trigger(new LoggedDashboardBoolean("keyboard/ctrl", false)::get);
        LEFT_ALT = new Trigger(new LoggedDashboardBoolean("keyboard/alt", false)::get);
        RIGHT_CONTROL = new Trigger(new LoggedDashboardBoolean("keyboard/right ctrl", false)::get);
        LEFT_ARROW = new Trigger(new LoggedDashboardBoolean("keyboard/left", false)::get);
        RIGHT_ARROW = new Trigger(new LoggedDashboardBoolean("keyboard/right", false)::get);
        UP_ARROW = new Trigger(new LoggedDashboardBoolean("keyboard/up", false)::get);
        DOWN_ARROW = new Trigger(new LoggedDashboardBoolean("keyboard/down", false)::get);
        NUMPAD_0 = new Trigger(new LoggedDashboardBoolean("keyboard/numpad0", false)::get);
        NUMPAD_1 = new Trigger(new LoggedDashboardBoolean("keyboard/numpad1", false)::get);
        NUMPAD_2 = new Trigger(new LoggedDashboardBoolean("keyboard/numpad2", false)::get);
        NUMPAD_3 = new Trigger(new LoggedDashboardBoolean("keyboard/numpad3", false)::get);
        NUMPAD_4 = new Trigger(new LoggedDashboardBoolean("keyboard/numpad4", false)::get);
        NUMPAD_5 = new Trigger(new LoggedDashboardBoolean("keyboard/numpad5", false)::get);
        NUMPAD_6 = new Trigger(new LoggedDashboardBoolean("keyboard/numpad6", false)::get);
        NUMPAD_7 = new Trigger(new LoggedDashboardBoolean("keyboard/numpad7", false)::get);
        NUMPAD_8 = new Trigger(new LoggedDashboardBoolean("keyboard/numpad8", false)::get);
        NUMPAD_9 = new Trigger(new LoggedDashboardBoolean("keyboard/numpad9", false)::get);
        keyListener();
    }

    private void keyListener() {

        try {
            GlobalScreen.registerNativeHook();
            GlobalScreen.addNativeKeyListener(new KeyboardListener());
        } catch (NativeHookException e) {
            throw new RuntimeException(e);
        }
    }

    public double getAxisValue(Controller.Axis axis) {
        return switch (axis) {
            case LEFT_X -> getValueByButtons(D,A);
            case LEFT_Y -> getValueByButtons(W,S);
            case LEFT_TRIGGER -> 0.0;
            case RIGHT_TRIGGER -> 0.0;
            case RIGHT_X -> getValueByButtons(E,Q);
            case RIGHT_Y -> getValueByButtons(X,Z);
        };
    }

    private static final double KEY_PRESSED_VALUE = 0.5;

    public double getValueByButtons(Trigger positiveValue, Trigger negativeValue) {
        if (positiveValue.getAsBoolean()) {
            return KEY_PRESSED_VALUE;
        }
        else if (negativeValue.getAsBoolean()) {
            return -KEY_PRESSED_VALUE;
        }
        else {
            return 0;
        }
    }
}
