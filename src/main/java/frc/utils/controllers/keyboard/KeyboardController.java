package frc.utils.controllers.keyboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.utils.applicationsutils.CMDHandler;
import frc.utils.dashboard.LoggedTableBoolean;


/*
 * very much inspired from Trigon code
 *
 * @author Yoni Kiriaty, Trigon
 *
 */
public class KeyboardController {

    private static final double KEY_PRESSED_VALUE = 0.5;

    private static final String KEYBOARD_TO_NETWORK_TABLES_SIMULATION_CLASS = "keyboard/keyboard_to_nt_simulation.py";

    private static final String KEYBOARD_TABLE = "Keyboard";
    private static final String KEYS_TAB = "Keys/";

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
        if (Robot.isSimulation() && RobotConstants.ENABLE_KEYBOARD) {
            CMDHandler.runPythonClass(KEYBOARD_TO_NETWORK_TABLES_SIMULATION_CLASS);
        }

        this.ESC = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "esc", false)::get);

        this.F1 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f1", false)::get);
        this.F2 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f2", false)::get);
        this.F3 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f3", false)::get);
        this.F4 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f4", false)::get);
        this.F5 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f5", false)::get);
        this.F6 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f6", false)::get);
        this.F7 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f7", false)::get);
        this.F8 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f8", false)::get);
        this.F9 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f9", false)::get);
        this.F10 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f10", false)::get);
        this.F11 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f11", false)::get);
        this.F12 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f12", false)::get);

        this.DELETE = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "delete", false)::get);
        this.BACKTICK = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "`", false)::get);

        this.ONE = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "1", false)::get);
        this.TWO = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "2", false)::get);
        this.THREE = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "3", false)::get);
        this.FOUR = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "4", false)::get);
        this.FIVE = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "5", false)::get);
        this.SIX = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "6", false)::get);
        this.SEVEN = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "7", false)::get);
        this.EIGHT = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "8", false)::get);
        this.NINE = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "9", false)::get);
        this.ZERO = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "0", false)::get);

        this.MINUS = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "-", false)::get);
        this.EQUALS = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "=", false)::get);
        this.BACKSPACE = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "backspace", false)::get);
        this.TAB = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "tab", false)::get);

        this.Q = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "q", false)::get);
        this.W = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "w", false)::get);
        this.E = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "e", false)::get);
        this.R = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "r", false)::get);
        this.T = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "t", false)::get);
        this.Y = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "y", false)::get);
        this.U = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "u", false)::get);
        this.I = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "i", false)::get);
        this.O = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "o", false)::get);
        this.P = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "p", false)::get);
        this.A = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "a", false)::get);
        this.S = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "s", false)::get);
        this.D = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "d", false)::get);
        this.F = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "f", false)::get);
        this.G = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "g", false)::get);
        this.H = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "h", false)::get);
        this.J = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "j", false)::get);
        this.K = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "k", false)::get);
        this.L = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "l", false)::get);
        this.Z = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "z", false)::get);
        this.X = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "x", false)::get);
        this.C = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "c", false)::get);
        this.V = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "v", false)::get);
        this.B = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "b", false)::get);
        this.N = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "n", false)::get);
        this.M = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "m", false)::get);

        this.SEMICOLON = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + ";", false)::get);
        this.APOSTROPHE = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "'", false)::get);
        this.LEFT_SHIFT = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "shift", false)::get);
        this.COMMA = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + ",", false)::get);
        this.PERIOD = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + ".", false)::get);
        this.SLASH = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "slash", false)::get);
        this.RIGHT_SHIFT = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "right shift", false)::get);
        this.LEFT_CONTROL = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "ctrl", false)::get);
        this.LEFT_ALT = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "alt", false)::get);
        this.RIGHT_CONTROL = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "right ctrl", false)::get);

        this.LEFT_ARROW = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "left", false)::get);
        this.RIGHT_ARROW = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "right", false)::get);
        this.UP_ARROW = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "up", false)::get);
        this.DOWN_ARROW = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "down", false)::get);

        this.NUMPAD_0 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "numpad0", false)::get);
        this.NUMPAD_1 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "numpad1", false)::get);
        this.NUMPAD_2 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "numpad2", false)::get);
        this.NUMPAD_3 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "numpad3", false)::get);
        this.NUMPAD_4 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "numpad4", false)::get);
        this.NUMPAD_5 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "numpad5", false)::get);
        this.NUMPAD_6 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "numpad6", false)::get);
        this.NUMPAD_7 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "numpad7", false)::get);
        this.NUMPAD_8 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "numpad8", false)::get);
        this.NUMPAD_9 = new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + "numpad9", false)::get);
    }

    public double getValueByButtons(Trigger positiveValue, Trigger negativeValue) {
        return getValueByButtons(positiveValue, negativeValue, KEY_PRESSED_VALUE);
    }

    public double getValueByButtons(Trigger positiveValue, Trigger negativeValue, double value) {
        if (positiveValue.getAsBoolean()) {
            return value;
        } else if (negativeValue.getAsBoolean()) {
            return -value;
        } else {
            return 0;
        }
    }

}
