package frc.joysticks;

import java.util.ArrayList;
import java.util.Arrays;

public enum BindSet {

	NO_JOYSTICK(0),
	NO_BINDINGS(1);

	private static final ArrayList<BindSet> bindSets = new ArrayList<>();
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

	public static void addBindSetsToHashMap() {
		bindSets.addAll(Arrays.asList(BindSet.values()));
	}

	public BindSet cycleBindSet(BindSet current, int jump) {
		int index = current.getIndex();

		if (index + jump > bindSets.size() - 1) {
			return bindSets.get(NO_BINDINGS.getIndex() + (index + jump - bindSets.size()) % (bindSets.size() - NO_BINDINGS.getIndex()));
		} else if (index + jump < NO_BINDINGS.getIndex()) {
			return bindSets.get(bindSets.size() + ((index + jump - NO_BINDINGS.getIndex()) % (bindSets.size() - NO_BINDINGS.getIndex())));
		} else {
			return bindSets.get(index + jump);
		}
	}

}
