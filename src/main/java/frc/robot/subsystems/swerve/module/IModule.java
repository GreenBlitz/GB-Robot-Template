package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.utils.calibration.sysid.SysIdCalibrator;


public interface IModule {

	SysIdCalibrator.SysIdConfigInfo getSteerSysIdConfigInfo();

	SysIdCalibrator.SysIdConfigInfo getDriveSysIdConfigInfo();

	void updateInputs();

	void setBrake(boolean brake);

	void resetByEncoder();

	void stop();

	void setDriveVoltage(double voltage);

	void setSteerVoltage(double voltage);


	public void pointSteer(Rotation2d steerTargetPosition, boolean optimize);


	public void setTargetState(SwerveModuleState targetState, boolean isClosedLoop);

	public void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerPosition, boolean isClosedLoop);


	/**
	 * The odometry thread can update itself faster than the main code loop (which is 50 hertz). Instead of using the latest odometry update, the
	 * accumulated odometry positions since the last loop to get a more accurate position.
	 *
	 * @param odometryUpdateIndex the index of the odometry update
	 * @return the position of the module at the given odometry update index
	 */
	public SwerveModulePosition getOdometryPosition(int odometryUpdateIndex);

	public int getNumberOfOdometrySamples();

	public SwerveModuleState getTargetState();

	public SwerveModuleState getCurrentState();

	public Rotation2d getDriveAngle();

	public double getDriveVelocityMetersPerSecond();

	public Rotation2d getSteerPosition();


	//@formatter:off
    public boolean isAtTargetVelocity(double speedToleranceMetersPerSecond);
    //@formatter:on

	public boolean isSteerAtTargetPosition(Rotation2d steerTolerance, Rotation2d steerVelocityPerSecondDeadband);

	public boolean isAtTargetState(Rotation2d steerTolerance, Rotation2d steerVelocityPerSecondDeadband, double speedToleranceMetersPerSecond);

}
