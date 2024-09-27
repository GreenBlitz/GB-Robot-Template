package frc.robot.subsystems.swerve;

public enum SwerveName {

	SWERVE("");

	private final String swerveName;

	SwerveName(String swerveName) {
		this.swerveName = swerveName;
	}

	public String getLogPath() {
		return "Subsystems/" + swerveName + "Swerve/";
	}

}
