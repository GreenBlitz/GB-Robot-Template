package frc.joysticks;

import java.util.HashMap;

public enum BindSet {

	NO_JOYSTICK(0),
	NO_BINDINGS(1),
	SWERVE(2),
	SECOND(3),
	TESTING(4);

	private final int index;

	BindSet(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

	public static HashMap<Integer, BindSet> bindSets = new HashMap<Integer, BindSet>();

	public static BindSet getBindSetByIndex(int index) {
		return switch (index) {
			case 0 -> NO_JOYSTICK;
			case 1 -> NO_BINDINGS;
			default -> NO_BINDINGS;
		};
	}

	public static void addBindSetsToHashMap() {
		for (BindSet bindSet : BindSet.values()) {
			bindSets.put(bindSet.getIndex(), bindSet);
		}
	}

}
