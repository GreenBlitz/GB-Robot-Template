package frc.utils.controllers.keyboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants;
import frc.utils.FileHandler;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;


/*
 * very much inspired from Trigon code
 *
 * @author Yoni Kiriaty, Trigon
 *
 */
public class KeyboardController {

    private static final double KEY_PRESSED_VALUE = 0.5;

    public final Trigger
            ESC, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10,
            F11, F12, DELETE, BACKTICK, ONE, TWO, THREE, FOUR,
            FIVE, SIX, SEVEN, EIGHT, NINE, ZERO, MINUS, EQUALS,
            BACKSPACE, TAB, Q, W, E, R, T, Y, U, I, O, P, A, S,
            D, F, G, H, J, K, L, SEMICOLON, APOSTROPHE, LEFT_SHIFT,
            Z, X, C, V, B, N, M, COMMA, PERIOD, SLASH,
            RIGHT_SHIFT, LEFT_CONTROL, LEFT_ALT, RIGHT_CONTROL,
            LEFT_ARROW, RIGHT_ARROW, UP_ARROW, DOWN_ARROW, NUMPAD_0, NUMPAD_1, NUMPAD_2,
            NUMPAD_3, NUMPAD_4, NUMPAD_5, NUMPAD_6, NUMPAD_7, NUMPAD_8,
            NUMPAD_9;

    /**
     * Construct an instance of a device.
     */
    public KeyboardController() {
        if (RobotConstants.ENABLE_KEYBOARD) {
            runKeyboardToNetworkTables();
        }

        this.ESC = new Trigger(new LoggedDashboardBoolean("Keyboard/esc", false)::get);
        this.F1 = new Trigger(new LoggedDashboardBoolean("Keyboard/f1", false)::get);
        this.F2 = new Trigger(new LoggedDashboardBoolean("Keyboard/f2", false)::get);
        this.F3 = new Trigger(new LoggedDashboardBoolean("Keyboard/f3", false)::get);
        this.F4 = new Trigger(new LoggedDashboardBoolean("Keyboard/f4", false)::get);
        this.F5 = new Trigger(new LoggedDashboardBoolean("Keyboard/f5", false)::get);
        this.F6 = new Trigger(new LoggedDashboardBoolean("Keyboard/f6", false)::get);
        this.F7 = new Trigger(new LoggedDashboardBoolean("Keyboard/f7", false)::get);
        this.F8 = new Trigger(new LoggedDashboardBoolean("Keyboard/f8", false)::get);
        this.F9 = new Trigger(new LoggedDashboardBoolean("Keyboard/f9", false)::get);
        this.F10 = new Trigger(new LoggedDashboardBoolean("Keyboard/f10", false)::get);
        this.F11 = new Trigger(new LoggedDashboardBoolean("Keyboard/f11", false)::get);
        this.F12 = new Trigger(new LoggedDashboardBoolean("Keyboard/f12", false)::get);

        this.DELETE = new Trigger(new LoggedDashboardBoolean("Keyboard/delete", false)::get);
        this.BACKTICK = new Trigger(new LoggedDashboardBoolean("Keyboard/`", false)::get);

        this.ONE = new Trigger(new LoggedDashboardBoolean("Keyboard/1", false)::get);
        this.TWO = new Trigger(new LoggedDashboardBoolean("Keyboard/2", false)::get);
        this.THREE = new Trigger(new LoggedDashboardBoolean("Keyboard/3", false)::get);
        this.FOUR = new Trigger(new LoggedDashboardBoolean("Keyboard/4", false)::get);
        this.FIVE = new Trigger(new LoggedDashboardBoolean("Keyboard/5", false)::get);
        this.SIX = new Trigger(new LoggedDashboardBoolean("Keyboard/6", false)::get);
        this.SEVEN = new Trigger(new LoggedDashboardBoolean("Keyboard/7", false)::get);
        this.EIGHT = new Trigger(new LoggedDashboardBoolean("Keyboard/8", false)::get);
        this.NINE = new Trigger(new LoggedDashboardBoolean("Keyboard/9", false)::get);
        this.ZERO = new Trigger(new LoggedDashboardBoolean("Keyboard/0", false)::get);
        this.MINUS = new Trigger(new LoggedDashboardBoolean("Keyboard/-", false)::get);
        this.EQUALS = new Trigger(new LoggedDashboardBoolean("Keyboard/=", false)::get);
        this.BACKSPACE = new Trigger(new LoggedDashboardBoolean("Keyboard/backspace", false)::get);
        this.TAB = new Trigger(new LoggedDashboardBoolean("Keyboard/tab", false)::get);

        this.Q = new Trigger(new LoggedDashboardBoolean("Keyboard/q", false)::get);
        this.W = new Trigger(new LoggedDashboardBoolean("Keyboard/w", false)::get);
        this.E = new Trigger(new LoggedDashboardBoolean("Keyboard/e", false)::get);
        this.R = new Trigger(new LoggedDashboardBoolean("Keyboard/r", false)::get);
        this.T = new Trigger(new LoggedDashboardBoolean("Keyboard/t", false)::get);
        this.Y = new Trigger(new LoggedDashboardBoolean("Keyboard/y", false)::get);
        this.U = new Trigger(new LoggedDashboardBoolean("Keyboard/u", false)::get);
        this.I = new Trigger(new LoggedDashboardBoolean("Keyboard/i", false)::get);
        this.O = new Trigger(new LoggedDashboardBoolean("Keyboard/o", false)::get);
        this.P = new Trigger(new LoggedDashboardBoolean("Keyboard/p", false)::get);
        this.A = new Trigger(new LoggedDashboardBoolean("Keyboard/a", false)::get);
        this.S = new Trigger(new LoggedDashboardBoolean("Keyboard/s", false)::get);
        this.D = new Trigger(new LoggedDashboardBoolean("Keyboard/d", false)::get);
        this.F = new Trigger(new LoggedDashboardBoolean("Keyboard/f", false)::get);
        this.G = new Trigger(new LoggedDashboardBoolean("Keyboard/g", false)::get);
        this.H = new Trigger(new LoggedDashboardBoolean("Keyboard/h", false)::get);
        this.J = new Trigger(new LoggedDashboardBoolean("Keyboard/j", false)::get);
        this.K = new Trigger(new LoggedDashboardBoolean("Keyboard/k", false)::get);
        this.L = new Trigger(new LoggedDashboardBoolean("Keyboard/l", false)::get);
        this.Z = new Trigger(new LoggedDashboardBoolean("Keyboard/z", false)::get);
        this.X = new Trigger(new LoggedDashboardBoolean("Keyboard/x", false)::get);
        this.C = new Trigger(new LoggedDashboardBoolean("Keyboard/c", false)::get);
        this.V = new Trigger(new LoggedDashboardBoolean("Keyboard/v", false)::get);
        this.B = new Trigger(new LoggedDashboardBoolean("Keyboard/b", false)::get);
        this.N = new Trigger(new LoggedDashboardBoolean("Keyboard/n", false)::get);
        this.M = new Trigger(new LoggedDashboardBoolean("Keyboard/m", false)::get);

        this.SEMICOLON = new Trigger(new LoggedDashboardBoolean("Keyboard/;", false)::get);
        this.APOSTROPHE = new Trigger(new LoggedDashboardBoolean("Keyboard/'", false)::get);
        this.LEFT_SHIFT = new Trigger(new LoggedDashboardBoolean("Keyboard/shift", false)::get);
        this.COMMA = new Trigger(new LoggedDashboardBoolean("Keyboard/,", false)::get);
        this.PERIOD = new Trigger(new LoggedDashboardBoolean("Keyboard/.", false)::get);
        this.SLASH = new Trigger(new LoggedDashboardBoolean("Keyboard/slash", false)::get);
        this.RIGHT_SHIFT = new Trigger(new LoggedDashboardBoolean("Keyboard/right shift", false)::get);
        this.LEFT_CONTROL = new Trigger(new LoggedDashboardBoolean("Keyboard/ctrl", false)::get);
        this.LEFT_ALT = new Trigger(new LoggedDashboardBoolean("Keyboard/alt", false)::get);
        this.RIGHT_CONTROL = new Trigger(new LoggedDashboardBoolean("Keyboard/right ctrl", false)::get);

        this.LEFT_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/left", false)::get);
        this.RIGHT_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/right", false)::get);
        this.UP_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/up", false)::get);
        this.DOWN_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/down", false)::get);

        this.NUMPAD_0 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad0", false)::get);
        this.NUMPAD_1 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad1", false)::get);
        this.NUMPAD_2 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad2", false)::get);
        this.NUMPAD_3 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad3", false)::get);
        this.NUMPAD_4 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad4", false)::get);
        this.NUMPAD_5 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad5", false)::get);
        this.NUMPAD_6 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad6", false)::get);
        this.NUMPAD_7 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad7", false)::get);
        this.NUMPAD_8 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad8", false)::get);
        this.NUMPAD_9 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad9", false)::get);
    }

    private void runKeyboardToNetworkTables() {
        FileHandler.runCmd(
                "py " +
                        FileHandler.getPathToPythonDirectory() +
                        "/keyboard_to_nt.py"
        );
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
