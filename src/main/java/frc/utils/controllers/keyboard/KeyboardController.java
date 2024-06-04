package frc.utils.controllers.keyboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.utils.controllers.Controller;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class KeyboardController implements Controller{

    public final Trigger
            ESC, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, DELETE,
            BACKTICK, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, ZERO, MINUS, EQUALS, BACKSPACE,
            TAB, Q, W, E, R, T, Y, U, I, O, P,
            A, S, D, F, G, H, J, K, L, SEMICOLON, APOSTROPHE,
            LEFT_SHIFT, Z, X, C, V, B, N, M, COMMA, PERIOD, RIGHT_SHIFT,
            LEFT_CONTROL, LEFT_ALT, LEFT_ARROW, RIGHT_ARROW, UP_ARROW, DOWN_ARROW;

    /**
     * Construct an instance of a device.
     */
    public KeyboardController() {
        ESC = new Trigger(new LoggedDashboardBoolean("Keyboard/Escape", false)::get);
        F1 = new Trigger(new LoggedDashboardBoolean("Keyboard/F1", false)::get);
        F2 = new Trigger(new LoggedDashboardBoolean("Keyboard/F2", false)::get);
        F3 = new Trigger(new LoggedDashboardBoolean("Keyboard/F3", false)::get);
        F4 = new Trigger(new LoggedDashboardBoolean("Keyboard/F4", false)::get);
        F5 = new Trigger(new LoggedDashboardBoolean("Keyboard/F5", false)::get);
        F6 = new Trigger(new LoggedDashboardBoolean("Keyboard/F6", false)::get);
        F7 = new Trigger(new LoggedDashboardBoolean("Keyboard/F7", false)::get);
        F8 = new Trigger(new LoggedDashboardBoolean("Keyboard/F8", false)::get);
        F9 = new Trigger(new LoggedDashboardBoolean("Keyboard/F9", false)::get);
        F10 = new Trigger(new LoggedDashboardBoolean("Keyboard/F10", false)::get);
        F11 = new Trigger(new LoggedDashboardBoolean("Keyboard/F11", false)::get);
        F12 = new Trigger(new LoggedDashboardBoolean("Keyboard/F12", false)::get);
        DELETE = new Trigger(new LoggedDashboardBoolean("Keyboard/Delete", false)::get);
        BACKTICK = new Trigger(new LoggedDashboardBoolean("Keyboard/Back Quote", false)::get);
        ONE = new Trigger(new LoggedDashboardBoolean("Keyboard/1", false)::get);
        TWO = new Trigger(new LoggedDashboardBoolean("Keyboard/2", false)::get);
        THREE = new Trigger(new LoggedDashboardBoolean("Keyboard/3", false)::get);
        FOUR = new Trigger(new LoggedDashboardBoolean("Keyboard/4", false)::get);
        FIVE = new Trigger(new LoggedDashboardBoolean("Keyboard/5", false)::get);
        SIX = new Trigger(new LoggedDashboardBoolean("Keyboard/6", false)::get);
        SEVEN = new Trigger(new LoggedDashboardBoolean("Keyboard/7", false)::get);
        EIGHT = new Trigger(new LoggedDashboardBoolean("Keyboard/8", false)::get);
        NINE = new Trigger(new LoggedDashboardBoolean("Keyboard/9", false)::get);
        ZERO = new Trigger(new LoggedDashboardBoolean("Keyboard/0", false)::get);
        MINUS = new Trigger(new LoggedDashboardBoolean("Keyboard/Minus", false)::get);
        EQUALS = new Trigger(new LoggedDashboardBoolean("Keyboard/Equals", false)::get);
        BACKSPACE = new Trigger(new LoggedDashboardBoolean("Keyboard/Backspace", false)::get);
        TAB = new Trigger(new LoggedDashboardBoolean("Keyboard/Tab", false)::get);
        Q = new Trigger(new LoggedDashboardBoolean("Keyboard/Q", false)::get);
        W = new Trigger(new LoggedDashboardBoolean("Keyboard/W", false)::get);
        E = new Trigger(new LoggedDashboardBoolean("Keyboard/E", false)::get);
        R = new Trigger(new LoggedDashboardBoolean("Keyboard/R", false)::get);
        T = new Trigger(new LoggedDashboardBoolean("Keyboard/T", false)::get);
        Y = new Trigger(new LoggedDashboardBoolean("Keyboard/Y", false)::get);
        U = new Trigger(new LoggedDashboardBoolean("Keyboard/U", false)::get);
        I = new Trigger(new LoggedDashboardBoolean("Keyboard/I", false)::get);
        O = new Trigger(new LoggedDashboardBoolean("Keyboard/O", false)::get);
        P = new Trigger(new LoggedDashboardBoolean("Keyboard/P", false)::get);
        A = new Trigger(new LoggedDashboardBoolean("Keyboard/A", false)::get);
        S = new Trigger(new LoggedDashboardBoolean("Keyboard/S", false)::get);
        D = new Trigger(new LoggedDashboardBoolean("Keyboard/D", false)::get);
        F = new Trigger(new LoggedDashboardBoolean("Keyboard/F", false)::get);
        G = new Trigger(new LoggedDashboardBoolean("Keyboard/G", false)::get);
        H = new Trigger(new LoggedDashboardBoolean("Keyboard/H", false)::get);
        J = new Trigger(new LoggedDashboardBoolean("Keyboard/J", false)::get);
        K = new Trigger(new LoggedDashboardBoolean("Keyboard/K", false)::get);
        L = new Trigger(new LoggedDashboardBoolean("Keyboard/L", false)::get);
        SEMICOLON = new Trigger(new LoggedDashboardBoolean("Keyboard/Semicolon", false)::get);
        APOSTROPHE = new Trigger(new LoggedDashboardBoolean("Keyboard/Quote", false)::get);
        LEFT_SHIFT = new Trigger(new LoggedDashboardBoolean("Keyboard/Shift", false)::get);
        Z = new Trigger(new LoggedDashboardBoolean("Keyboard/Z", false)::get);
        X = new Trigger(new LoggedDashboardBoolean("Keyboard/X", false)::get);
        C = new Trigger(new LoggedDashboardBoolean("Keyboard/C", false)::get);
        V = new Trigger(new LoggedDashboardBoolean("Keyboard/V", false)::get);
        B = new Trigger(new LoggedDashboardBoolean("Keyboard/B", false)::get);
        N = new Trigger(new LoggedDashboardBoolean("Keyboard/N", false)::get);
        M = new Trigger(new LoggedDashboardBoolean("Keyboard/M", false)::get);
        COMMA = new Trigger(new LoggedDashboardBoolean("Keyboard/Comma", false)::get);
        PERIOD = new Trigger(new LoggedDashboardBoolean("Keyboard/Period", false)::get);
        RIGHT_SHIFT = new Trigger(new LoggedDashboardBoolean("Keyboard/Unknown keyCode: 0xe36", false)::get);
        LEFT_CONTROL = new Trigger(new LoggedDashboardBoolean("Keyboard/Ctrl", false)::get);
        LEFT_ALT = new Trigger(new LoggedDashboardBoolean("Keyboard/Alt", false)::get);
        LEFT_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/Left", false)::get);
        RIGHT_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/Right", false)::get);
        UP_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/Up", false)::get);
        DOWN_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/Down", false)::get);
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
