package frc.joysticks;

public enum BindSet {

	NO_JOYSTICK(0),
	NO_BINDINGS(1),
	SWERVE(2),
	SECOND(3);

	private final int index;

	BindSet(int index) {
		this.index = index;
	}

	public int getIndex() {
		return index;
	}

	public static BindSet cycleBindSet(BindSet current, int jump) {
		int index = current.getIndex();
		BindSet[] bindSets = values();
		System.out.println("Aaaaa"+current);
		if (index + jump > bindSets.length - 1) {
			System.out.println(1);
			//System.out.println(bindSets[NO_BINDINGS.getIndex() + (index + jump - bindSets.length) % (bindSets.length - NO_BINDINGS.getIndex())].name());
			return bindSets[NO_BINDINGS.getIndex() + (index + jump - bindSets.length) % (bindSets.length - NO_BINDINGS.getIndex())];
		} else if (index + jump < NO_BINDINGS.getIndex()) {
			System.out.println(2);
			//System.out.println(bindSets[bindSets.length + ((index + jump - NO_BINDINGS.getIndex()) % (bindSets.length - NO_BINDINGS.getIndex()))].name());
			return bindSets[bindSets.length + ((index + jump - NO_BINDINGS.getIndex()) % (bindSets.length - NO_BINDINGS.getIndex()))];
		} else {
			//System.out.println(bindSets[index + jump].name());
			System.out.println(3);
			return bindSets[index + jump];
		}
	}

}
