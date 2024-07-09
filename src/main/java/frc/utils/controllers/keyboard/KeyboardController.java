package frc.utils.controllers.keyboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotManager;
import frc.robot.constants.IPConstants;
import frc.utils.CMDHandler;
import frc.utils.dashboard.LoggedTableBoolean;

import java.nio.file.Path;


/*
 * Inspired from Trigon code.
 *
 * @author Yoni Kiriaty, Trigon
 */
public class KeyboardController {

    private static final double KEY_PRESSED_VALUE = 0.5;

    private static final Path KEYBOARD_TO_NETWORK_TABLES_CLASS = Path.of("KeyboardToNetworkTables");

    private static final String KEYBOARD_TABLE = "Keyboard";
    private static final String KEYS_TAB = "Keys/";

    public final Trigger
            ESC,
            F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12,
            DELETE, BACKTICK,
            ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, ZERO,
            MINUS, EQUALS, BACKSPACE, TAB,
            A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
            SEMICOLON, APOSTROPHE, LEFT_SHIFT, COMMA, PERIOD, SLASH, RIGHT_SHIFT, LEFT_CONTROL, LEFT_ALT, RIGHT_CONTROL,
            LEFT_ARROW, RIGHT_ARROW, UP_ARROW, DOWN_ARROW,
            NUMPAD_0, NUMPAD_1, NUMPAD_2, NUMPAD_3, NUMPAD_4, NUMPAD_5, NUMPAD_6, NUMPAD_7, NUMPAD_8, NUMPAD_9;

    public KeyboardController() {
        if (RobotManager.isSimulation()) {
            CMDHandler.runPythonClass(KEYBOARD_TO_NETWORK_TABLES_CLASS, IPConstants.SIMULATION_IP);
        }

        this.ESC = getNewKey("esc");

        this.F1 = getNewKey("f1");
        this.F2 = getNewKey("f2");
        this.F3 = getNewKey("f3");
        this.F4 = getNewKey("f4");
        this.F5 = getNewKey("f5");
        this.F6 = getNewKey("f6");
        this.F7 = getNewKey("f7");
        this.F8 = getNewKey("f8");
        this.F9 = getNewKey("f9");
        this.F10 = getNewKey("f10");
        this.F11 = getNewKey("f11");
        this.F12 = getNewKey("f12");

        this.DELETE = getNewKey("delete");
        this.BACKTICK = getNewKey("`");

        this.ONE = getNewKey("1");
        this.TWO = getNewKey("2");
        this.THREE = getNewKey("3");
        this.FOUR = getNewKey("4");
        this.FIVE = getNewKey("5");
        this.SIX = getNewKey("6");
        this.SEVEN = getNewKey("7");
        this.EIGHT = getNewKey("8");
        this.NINE = getNewKey("9");
        this.ZERO = getNewKey("0");

        this.MINUS = getNewKey("-");
        this.EQUALS = getNewKey("=");
        this.BACKSPACE = getNewKey("backspace");
        this.TAB = getNewKey("tab");

        this.A = getNewKey("a");
        this.B = getNewKey("b");
        this.C = getNewKey("c");
        this.D = getNewKey("d");
        this.E = getNewKey("e");
        this.F = getNewKey("f");
        this.G = getNewKey("g");
        this.H = getNewKey("h");
        this.I = getNewKey("i");
        this.J = getNewKey("j");
        this.K = getNewKey("k");
        this.L = getNewKey("l");
        this.M = getNewKey("m");
        this.N = getNewKey("n");
        this.O = getNewKey("o");
        this.P = getNewKey("p");
        this.Q = getNewKey("q");
        this.R = getNewKey("r");
        this.S = getNewKey("s");
        this.T = getNewKey("t");
        this.U = getNewKey("u");
        this.V = getNewKey("v");
        this.W = getNewKey("w");
        this.X = getNewKey("x");
        this.Y = getNewKey("y");
        this.Z = getNewKey("z");

        this.SEMICOLON = getNewKey(";");
        this.APOSTROPHE = getNewKey("'");
        this.LEFT_SHIFT = getNewKey("shift");
        this.COMMA = getNewKey(",");
        this.PERIOD = getNewKey(".");
        this.SLASH = getNewKey("slash");
        this.RIGHT_SHIFT = getNewKey("right shift");
        this.LEFT_CONTROL = getNewKey("ctrl");
        this.LEFT_ALT = getNewKey("alt");
        this.RIGHT_CONTROL = getNewKey("right ctrl");

        this.LEFT_ARROW = getNewKey("left");
        this.RIGHT_ARROW = getNewKey("right");
        this.UP_ARROW = getNewKey("up");
        this.DOWN_ARROW = getNewKey("down");

        this.NUMPAD_0 = getNewKey("numpad0");
        this.NUMPAD_1 = getNewKey("numpad1");
        this.NUMPAD_2 = getNewKey("numpad2");
        this.NUMPAD_3 = getNewKey("numpad3");
        this.NUMPAD_4 = getNewKey("numpad4");
        this.NUMPAD_5 = getNewKey("numpad5");
        this.NUMPAD_6 = getNewKey("numpad6");
        this.NUMPAD_7 = getNewKey("numpad7");
        this.NUMPAD_8 = getNewKey("numpad8");
        this.NUMPAD_9 = getNewKey("numpad9");
    }

    private Trigger getNewKey(String name) {
        return new Trigger(new LoggedTableBoolean(KEYBOARD_TABLE, KEYS_TAB + name, false)::get);
    }

    public double getValueByButtons(Trigger positiveButton, Trigger negativeButton) {
        return getValueByButtons(positiveButton, negativeButton, KEY_PRESSED_VALUE);
    }

    public double getValueByButtons(Trigger positiveButton, Trigger negativeButton, double value) {
        if (positiveButton.getAsBoolean()) {
            return value;
        }
        else if (negativeButton.getAsBoolean()) {
            return -value;
        }
        else {
            return 0;
        }
    }

}
