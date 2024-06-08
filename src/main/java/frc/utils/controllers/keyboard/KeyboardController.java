package frc.utils.controllers.keyboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.utils.CMDHandler;
import frc.utils.dashboard.LoggedTableBoolean;


/*
 * very much inspired from Trigon code
 *
 * @author Yoni Kiriaty, Trigon
 *
 */
public class KeyboardController {

    private static final double KEY_PRESSED_VALUE = 0.5;

    private static final String KEYBOARD_TO_NETWORK_TABLES_PATH = "py " + CMDHandler.PATH_TO_PYTHON_DIRECTORY + "/keyboard_to_nt.py";

    public final Trigger
            ESC,
            F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12,
            DELETE, BACKTICK,
            ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, ZERO,
            MINUS, EQUALS,
            BACKSPACE, TAB,
            Q, W, E, R, T, Y, U, I, O, P, A, S, D, F, G, H, J, K, L, Z, X, C, V, B, N, M,
            SEMICOLON, APOSTROPHE, LEFT_SHIFT, COMMA, PERIOD, SLASH, RIGHT_SHIFT, LEFT_CONTROL, LEFT_ALT, RIGHT_CONTROL,
            LEFT_ARROW, RIGHT_ARROW, UP_ARROW, DOWN_ARROW,
            NUMPAD_0, NUMPAD_1, NUMPAD_2, NUMPAD_3, NUMPAD_4, NUMPAD_5, NUMPAD_6, NUMPAD_7, NUMPAD_8, NUMPAD_9;

    public KeyboardController() {
        CMDHandler.runCMDCommand(KEYBOARD_TO_NETWORK_TABLES_PATH);

        this.ESC = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/esc", false)::get);

        this.F1 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f1", false)::get);
        this.F2 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f2", false)::get);
        this.F3 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f3", false)::get);
        this.F4 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f4", false)::get);
        this.F5 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f5", false)::get);
        this.F6 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f6", false)::get);
        this.F7 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f7", false)::get);
        this.F8 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f8", false)::get);
        this.F9 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f9", false)::get);
        this.F10 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f10", false)::get);
        this.F11 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f11", false)::get);
        this.F12 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f12", false)::get);

        this.DELETE = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/delete", false)::get);
        this.BACKTICK = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/`", false)::get);

        this.ONE = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/1", false)::get);
        this.TWO = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/2", false)::get);
        this.THREE = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/3", false)::get);
        this.FOUR = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/4", false)::get);
        this.FIVE = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/5", false)::get);
        this.SIX = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/6", false)::get);
        this.SEVEN = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/7", false)::get);
        this.EIGHT = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/8", false)::get);
        this.NINE = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/9", false)::get);
        this.ZERO = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/0", false)::get);

        this.MINUS = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/-", false)::get);
        this.EQUALS = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/=", false)::get);
        this.BACKSPACE = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/backspace", false)::get);
        this.TAB = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/tab", false)::get);

        this.Q = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/q", false)::get);
        this.W = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/w", false)::get);
        this.E = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/e", false)::get);
        this.R = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/r", false)::get);
        this.T = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/t", false)::get);
        this.Y = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/y", false)::get);
        this.U = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/u", false)::get);
        this.I = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/i", false)::get);
        this.O = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/o", false)::get);
        this.P = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/p", false)::get);
        this.A = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/a", false)::get);
        this.S = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/s", false)::get);
        this.D = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/d", false)::get);
        this.F = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/f", false)::get);
        this.G = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/g", false)::get);
        this.H = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/h", false)::get);
        this.J = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/j", false)::get);
        this.K = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/k", false)::get);
        this.L = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/l", false)::get);
        this.Z = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/z", false)::get);
        this.X = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/x", false)::get);
        this.C = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/c", false)::get);
        this.V = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/v", false)::get);
        this.B = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/b", false)::get);
        this.N = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/n", false)::get);
        this.M = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/m", false)::get);

        this.SEMICOLON = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/;", false)::get);
        this.APOSTROPHE = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/'", false)::get);
        this.LEFT_SHIFT = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/shift", false)::get);
        this.COMMA = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/,", false)::get);
        this.PERIOD = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/.", false)::get);
        this.SLASH = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/slash", false)::get);
        this.RIGHT_SHIFT = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/right shift", false)::get);
        this.LEFT_CONTROL = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/ctrl", false)::get);
        this.LEFT_ALT = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/alt", false)::get);
        this.RIGHT_CONTROL = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/right ctrl", false)::get);

        this.LEFT_ARROW = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/left", false)::get);
        this.RIGHT_ARROW = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/right", false)::get);
        this.UP_ARROW = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/up", false)::get);
        this.DOWN_ARROW = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/down", false)::get);

        this.NUMPAD_0 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/numpad0", false)::get);
        this.NUMPAD_1 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/numpad1", false)::get);
        this.NUMPAD_2 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/numpad2", false)::get);
        this.NUMPAD_3 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/numpad3", false)::get);
        this.NUMPAD_4 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/numpad4", false)::get);
        this.NUMPAD_5 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/numpad5", false)::get);
        this.NUMPAD_6 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/numpad6", false)::get);
        this.NUMPAD_7 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/numpad7", false)::get);
        this.NUMPAD_8 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/numpad8", false)::get);
        this.NUMPAD_9 = new Trigger(new LoggedTableBoolean("Keyboard", "Keys/numpad9", false)::get);
    }

    public double getValueByButtons(Trigger positiveValue, Trigger negativeValue) {
        return getValueByButtons(positiveValue, negativeValue, KEY_PRESSED_VALUE);
    }

    public double getValueByButtons(Trigger positiveValue, Trigger negativeValue, double value) {
        if (positiveValue.getAsBoolean()) {
            return value;
        }
        else if (negativeValue.getAsBoolean()) {
            return -value;
        }
        else {
            return 0;
        }
    }

}
