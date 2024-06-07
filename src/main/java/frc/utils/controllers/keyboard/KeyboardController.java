package frc.utils.controllers.keyboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import java.io.IOException;
import java.nio.file.Path;

/*
* very much inspired from Trigon code
*
* @author Yoni Kiriaty, Trigon
*
 */
public class KeyboardController{

    private static final double KEY_PRESSED_VALUE = 0.5;

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
        if (Robot.isSimulation()) {
            runKeyboardToNetworkTables();
        }

        ESC = new Trigger(new LoggedDashboardBoolean("Keyboard/esc", false)::get);
        F1 = new Trigger(new LoggedDashboardBoolean("Keyboard/f1", false)::get);
        F2 = new Trigger(new LoggedDashboardBoolean("Keyboard/f2", false)::get);
        F3 = new Trigger(new LoggedDashboardBoolean("Keyboard/f3", false)::get);
        F4 = new Trigger(new LoggedDashboardBoolean("Keyboard/f4", false)::get);
        F5 = new Trigger(new LoggedDashboardBoolean("Keyboard/f5", false)::get);
        F6 = new Trigger(new LoggedDashboardBoolean("Keyboard/f6", false)::get);
        F7 = new Trigger(new LoggedDashboardBoolean("Keyboard/f7", false)::get);
        F8 = new Trigger(new LoggedDashboardBoolean("Keyboard/f8", false)::get);
        F9 = new Trigger(new LoggedDashboardBoolean("Keyboard/f9", false)::get);
        F10 = new Trigger(new LoggedDashboardBoolean("Keyboard/f10", false)::get);
        F11 = new Trigger(new LoggedDashboardBoolean("Keyboard/f11", false)::get);
        F12 = new Trigger(new LoggedDashboardBoolean("Keyboard/f12", false)::get);
        DELETE = new Trigger(new LoggedDashboardBoolean("Keyboard/delete", false)::get);
        BACKTICK = new Trigger(new LoggedDashboardBoolean("Keyboard/`", false)::get);
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
        MINUS = new Trigger(new LoggedDashboardBoolean("Keyboard/-", false)::get);
        EQUALS = new Trigger(new LoggedDashboardBoolean("Keyboard/=", false)::get);
        BACKSPACE = new Trigger(new LoggedDashboardBoolean("Keyboard/backspace", false)::get);
        TAB = new Trigger(new LoggedDashboardBoolean("Keyboard/tab", false)::get);
        Q = new Trigger(new LoggedDashboardBoolean("Keyboard/q", false)::get);
        W = new Trigger(new LoggedDashboardBoolean("Keyboard/w", false)::get);
        E = new Trigger(new LoggedDashboardBoolean("Keyboard/e", false)::get);
        R = new Trigger(new LoggedDashboardBoolean("Keyboard/r", false)::get);
        T = new Trigger(new LoggedDashboardBoolean("Keyboard/t", false)::get);
        Y = new Trigger(new LoggedDashboardBoolean("Keyboard/y", false)::get);
        U = new Trigger(new LoggedDashboardBoolean("Keyboard/u", false)::get);
        I = new Trigger(new LoggedDashboardBoolean("Keyboard/i", false)::get);
        O = new Trigger(new LoggedDashboardBoolean("Keyboard/o", false)::get);
        P = new Trigger(new LoggedDashboardBoolean("Keyboard/p", false)::get);
        A = new Trigger(new LoggedDashboardBoolean("Keyboard/a", false)::get);
        S = new Trigger(new LoggedDashboardBoolean("Keyboard/s", false)::get);
        D = new Trigger(new LoggedDashboardBoolean("Keyboard/d", false)::get);
        F = new Trigger(new LoggedDashboardBoolean("Keyboard/f", false)::get);
        G = new Trigger(new LoggedDashboardBoolean("Keyboard/g", false)::get);
        H = new Trigger(new LoggedDashboardBoolean("Keyboard/h", false)::get);
        J = new Trigger(new LoggedDashboardBoolean("Keyboard/j", false)::get);
        K = new Trigger(new LoggedDashboardBoolean("Keyboard/k", false)::get);
        L = new Trigger(new LoggedDashboardBoolean("Keyboard/l", false)::get);
        SEMICOLON = new Trigger(new LoggedDashboardBoolean("Keyboard/;", false)::get);
        APOSTROPHE = new Trigger(new LoggedDashboardBoolean("Keyboard/'", false)::get);
        LEFT_SHIFT = new Trigger(new LoggedDashboardBoolean("Keyboard/shift", false)::get);
        Z = new Trigger(new LoggedDashboardBoolean("Keyboard/z", false)::get);
        X = new Trigger(new LoggedDashboardBoolean("Keyboard/x", false)::get);
        C = new Trigger(new LoggedDashboardBoolean("Keyboard/c", false)::get);
        V = new Trigger(new LoggedDashboardBoolean("Keyboard/v", false)::get);
        B = new Trigger(new LoggedDashboardBoolean("Keyboard/b", false)::get);
        N = new Trigger(new LoggedDashboardBoolean("Keyboard/n", false)::get);
        M = new Trigger(new LoggedDashboardBoolean("Keyboard/m", false)::get);
        COMMA = new Trigger(new LoggedDashboardBoolean("Keyboard/,", false)::get);
        PERIOD = new Trigger(new LoggedDashboardBoolean("Keyboard/.", false)::get);
        RIGHT_SHIFT = new Trigger(new LoggedDashboardBoolean("Keyboard/right shift", false)::get);
        LEFT_CONTROL = new Trigger(new LoggedDashboardBoolean("Keyboard/ctrl", false)::get);
        LEFT_ALT = new Trigger(new LoggedDashboardBoolean("Keyboard/alt", false)::get);
        RIGHT_CONTROL = new Trigger(new LoggedDashboardBoolean("Keyboard/right ctrl", false)::get);
        LEFT_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/left", false)::get);
        RIGHT_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/right", false)::get);
        UP_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/up", false)::get);
        DOWN_ARROW = new Trigger(new LoggedDashboardBoolean("Keyboard/down", false)::get);
        NUMPAD_0 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad0", false)::get);
        NUMPAD_1 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad1", false)::get);
        NUMPAD_2 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad2", false)::get);
        NUMPAD_3 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad3", false)::get);
        NUMPAD_4 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad4", false)::get);
        NUMPAD_5 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad5", false)::get);
        NUMPAD_6 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad6", false)::get);
        NUMPAD_7 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad7", false)::get);
        NUMPAD_8 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad8", false)::get);
        NUMPAD_9 = new Trigger(new LoggedDashboardBoolean("Keyboard/numpad9", false)::get);
    }

    public void runKeyboardToNetworkTables() {
        Runtime rt = Runtime.getRuntime();
        try {
            System.out.println("py " + getPathToPython() + "keyboard_to_nt.py");
            rt.exec(new String[]{"cmd.exe","/c","py " + getPathToPython() + "keyboard_to_nt.py"});

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String getPathToPython() {
        String repoPath = Path.of("").toAbsolutePath().toString();
        String pythonPath = repoPath + "/src/main/python/";

        return pythonPath;
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

    public double getValueByButtons(Trigger positiveValue, Trigger negativeValue) {
        return getValueByButtons(positiveValue, negativeValue, KEY_PRESSED_VALUE);
    }
}
