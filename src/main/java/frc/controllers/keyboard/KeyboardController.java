package frc.controllers.keyboard;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.IPs;
import frc.robot.Robot;
import frc.utils.CMDHandler;

import java.nio.file.Path;

/*
 * Inspired from Trigon code.
 *
 * @author Yoni Kiriaty
 */
public class KeyboardController {

	public static final boolean ENABLE_KEYBOARD = true;

	private static final double KEY_PRESSED_DEFAULT_VALUE = 0.5;
	private static final Path KEYBOARD_TO_NETWORK_TABLES_CLASS = Path.of("KeyboardToNetworkTables");

	private static final String KEYBOARD_TABLE = "Keyboard";
	private static final String KEYS_TAB = "Keys";

	public final Trigger ESC, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, DELETE, BACKTICK, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN,
		EIGHT, NINE, ZERO, MINUS, EQUALS, SPACE, BACKSPACE, TAB, A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
		SEMICOLON, APOSTROPHE, LEFT_SHIFT, COMMA, PERIOD, SLASH, RIGHT_SHIFT, LEFT_CONTROL, LEFT_ALT, RIGHT_CONTROL, LEFT_ARROW, RIGHT_ARROW,
		UP_ARROW, DOWN_ARROW;

	public KeyboardController() {
		if (Robot.ROBOT_TYPE.isSimulation()) {
			CMDHandler.runPythonClass(KEYBOARD_TO_NETWORK_TABLES_CLASS, IPs.SIMULATION_IP);
		}

		this.ESC = getKey("esc");

		this.F1 = getKey("f1");
		this.F2 = getKey("f2");
		this.F3 = getKey("f3");
		this.F4 = getKey("f4");
		this.F5 = getKey("f5");
		this.F6 = getKey("f6");
		this.F7 = getKey("f7");
		this.F8 = getKey("f8");
		this.F9 = getKey("f9");
		this.F10 = getKey("f10");
		this.F11 = getKey("f11");
		this.F12 = getKey("f12");

		this.DELETE = getKey("delete");
		this.BACKTICK = getKey("`");

		this.ONE = getKey("1");
		this.TWO = getKey("2");
		this.THREE = getKey("3");
		this.FOUR = getKey("4");
		this.FIVE = getKey("5");
		this.SIX = getKey("6");
		this.SEVEN = getKey("7");
		this.EIGHT = getKey("8");
		this.NINE = getKey("9");
		this.ZERO = getKey("0");

		this.MINUS = getKey("-");
		this.EQUALS = getKey("=");
		this.SPACE = getKey("space");
		this.BACKSPACE = getKey("backspace");
		this.TAB = getKey("tab");

		this.A = getKey("a");
		this.B = getKey("b");
		this.C = getKey("c");
		this.D = getKey("d");
		this.E = getKey("e");
		this.F = getKey("f");
		this.G = getKey("g");
		this.H = getKey("h");
		this.I = getKey("i");
		this.J = getKey("j");
		this.K = getKey("k");
		this.L = getKey("l");
		this.M = getKey("m");
		this.N = getKey("n");
		this.O = getKey("o");
		this.P = getKey("p");
		this.Q = getKey("q");
		this.R = getKey("r");
		this.S = getKey("s");
		this.T = getKey("t");
		this.U = getKey("u");
		this.V = getKey("v");
		this.W = getKey("w");
		this.X = getKey("x");
		this.Y = getKey("y");
		this.Z = getKey("z");

		this.SEMICOLON = getKey(";");
		this.APOSTROPHE = getKey("'");
		this.LEFT_SHIFT = getKey("shift");
		this.COMMA = getKey(",");
		this.PERIOD = getKey(".");
		this.SLASH = getKey("slash");
		this.RIGHT_SHIFT = getKey("shift_r");
		this.LEFT_CONTROL = getKey("ctrl_l");
		this.LEFT_ALT = getKey("alt_l");
		this.RIGHT_CONTROL = getKey("ctrl_r");

		this.LEFT_ARROW = getKey("left");
		this.RIGHT_ARROW = getKey("right");
		this.UP_ARROW = getKey("up");
		this.DOWN_ARROW = getKey("down");
	}

	private Trigger getKey(String name) {
		return new Trigger(
			NetworkTableInstance.getDefault().getTable(KEYBOARD_TABLE).getBooleanTopic(KEYS_TAB + "/" + name).subscribe(false)::get
		);
	}

	public double getValueByButtons(Trigger positiveButton, Trigger negativeButton) {
		return getValueByButtons(positiveButton, negativeButton, KEY_PRESSED_DEFAULT_VALUE);
	}

	public double getValueByButtons(Trigger positiveButton, Trigger negativeButton, double value) {
		if (positiveButton.getAsBoolean()) {
			return value;
		} else if (negativeButton.getAsBoolean()) {
			return -value;
		} else {
			return 0;
		}
	}

}
