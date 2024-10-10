
package frc.robot.subsystems.swerve;

public enum SwerveType {

	SWERVE("");

	private final String swerveName;

	SwerveType(String swerveName) {
		this.swerveName = swerveName;
	}

	public String getLogPath() {
		return "Subsystems/" + swerveName + "Swerve/";
	}

}
