package frc.joysticks;

import java.util.ArrayList;
import java.util.Arrays;

public enum BindSet {

	NO_JOYSTICK(0),
	NO_BINDINGS(1);

	private static final ArrayList<BindSet> bindSets = new ArrayList<>(Arrays.asList(BindSet.values()));
	private final int index;

	BindSet(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

	public static ArrayList<BindSet> getBindSets() {
		return bindSets;
	}

}
