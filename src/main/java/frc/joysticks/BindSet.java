package frc.joysticks;

public enum BindSet {

	NO_JOYSTICK(0),
	NO_BINDINGS(1);

	private final int index;

	BindSet(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

	public BindSet cycleBindSet(BindSet current, int jump) {
		int index = current.getIndex();
		BindSet[] bindSets = values();

		if (index + jump > bindSets.length - 1) {
			return bindSets[NO_BINDINGS.getIndex() + (index + jump - bindSets.length) % (bindSets.length - NO_BINDINGS.getIndex())];
		} else if (index + jump < NO_BINDINGS.getIndex()) {
			return bindSets[bindSets.length + ((index + jump - NO_BINDINGS.getIndex()) % (bindSets.length - NO_BINDINGS.getIndex()))];
		} else {
			return bindSets[index + jump];
		}
	}

}
