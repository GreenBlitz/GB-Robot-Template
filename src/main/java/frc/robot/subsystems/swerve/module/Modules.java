package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class Modules {

	private final Module[] modules;
	private final String logPath;
	private final double[] lastCycleLimitedTargetVelocities;

	public Modules(String logPath, Module... modules) {
		this.logPath = logPath + ModuleConstants.MODULES_LOG_PATH_ADDITION;
		this.modules = modules;
		this.lastCycleLimitedTargetVelocities = new double[modules.length];
	}

	public Module getModule(ModuleUtil.ModulePosition modulePosition) {
		return modules[modulePosition.getIndex()];
	}


	public void updateInputs() {
		for (Module currentModule : modules) {
			currentModule.updateInputs();
		}
		Logger.recordOutput(logPath + "/CurrentStates", getCurrentStates());
		Logger.recordOutput(logPath + "/TargetStates", getTargetStates());
	}


	public void resetModulesAngleByEncoder() {
		for (Module module : modules) {
			module.resetSteerByEncoder();
		}
	}

	public void setBrake(boolean brake) {
		for (Module currentModule : modules) {
			currentModule.setBrake(brake);
		}
	}


	public void pointWheels(Rotation2d targetSteerPosition, boolean optimize) {
		for (Module module : modules) {
			module.pointSteer(targetSteerPosition, optimize);
		}
	}

	public void pointWheelsInCircle() {
		Arrays.stream(modules).forEach(Module::pointInCircle);
	}

	public void pointWheelsInX() {
		Arrays.stream(modules).forEach(Module::pointToCenter);
	}


	public void stop() {
		for (Module currentModule : modules) {
			currentModule.stop();
		}
	}

	public void setSteersVoltage(double voltage) {
		for (Module module : modules) {
			module.setSteerVoltage(voltage);
		}
	}

	public void setDrivesCurrent(double current) {
		for (Module module : modules) {
			module.setDriveCurrent(current);
		}
	}

	public void setDrivesVoltage(double voltage) {
		for (Module module : modules) {
			module.setDriveVoltage(voltage);
		}
	}

	public void setTargetStates(SwerveModuleState[] moduleStates, boolean isClosedLoop, SlewRateLimiter slewRateLimiter) {
		double[] targetLimitedAccelerationsMetersPerSecondSquared = new double[modules.length];
		for (int i = 0; i < modules.length; i++) {
			double limitedVelocityMetersPerSecond = slewRateLimiter.calculate(moduleStates[i].speedMetersPerSecond);
			moduleStates[i].speedMetersPerSecond = limitedVelocityMetersPerSecond;
			targetLimitedAccelerationsMetersPerSecondSquared[i] = (limitedVelocityMetersPerSecond - lastCycleLimitedTargetVelocities[i])
				/ TimeUtil.DEFAULT_CYCLE_TIME_SECONDS;
			lastCycleLimitedTargetVelocities[i] = limitedVelocityMetersPerSecond;
			modules[i].setTargetState(moduleStates[i], isClosedLoop);
		}
		Logger.recordOutput(logPath + "/TargetAccelerationMetersPerSecondsSquared", targetLimitedAccelerationsMetersPerSecondSquared);
	}


	public Translation2d[] getModulePositionsFromCenterMeters() {
		return Arrays.stream(modules).map(Module::getPositionFromCenterMeters).toArray(Translation2d[]::new);
	}

	public int getNumberOfOdometrySamples() {
		int numberOfOdometrySamples = modules[0].getNumberOfOdometrySamples();
		for (int i = 1; i < modules.length; i++) {
			numberOfOdometrySamples = Math.min(numberOfOdometrySamples, modules[i].getNumberOfOdometrySamples());
		}
		return numberOfOdometrySamples;
	}

	public double[] getOdometryTimestamps() {
		int numberOfOdometrySamples = getNumberOfOdometrySamples();
		double[] timestamps = new double[numberOfOdometrySamples];
		for (int i = 0; i < numberOfOdometrySamples; i++) {
			timestamps[i] = modules[0].getOdometryTimestamp(i);
		}
		return timestamps;
	}

	public boolean isConnected() {
		for (Module module : modules) {
			if (!module.isConnected()) {
				return false;
			}
		}
		return true;
	}

	public SwerveModuleState[] getTargetStates() {
		return Arrays.stream(modules).map(Module::getTargetState).toArray(SwerveModuleState[]::new);
	}

	public SwerveModuleState[] getCurrentStates() {
		return Arrays.stream(modules).map(Module::getCurrentState).toArray(SwerveModuleState[]::new);
	}

	public Rotation2d[] getDrivesPositions() {
		return Arrays.stream(modules).map(Module::getDrivePosition).toArray(Rotation2d[]::new);
	}

	public SwerveModulePosition[] getWheelPositions(int odometrySampleIndex) {
		SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			swerveModulePositions[i] = modules[i].getOdometryPosition(odometrySampleIndex);
		}
		return swerveModulePositions;
	}


	public boolean isAtTargetVelocities(double speedToleranceMetersPerSecond) {
		for (Module module : modules) {
			if (!module.isAtTargetVelocity(speedToleranceMetersPerSecond)) {
				return false;
			}
		}
		return true;
	}

	public boolean isSteerAtTargetPositions(Rotation2d steerPositionTolerance, Rotation2d steerVelocityPerSecondDeadband) {
		for (Module module : modules) {
			if (!module.isSteerAtTargetPosition(steerPositionTolerance, steerVelocityPerSecondDeadband)) {
				return false;
			}
		}
		return true;
	}

	public boolean isAtTargetStates(
		Rotation2d steerPositionTolerance,
		Rotation2d steerVelocityPerSecondDeadband,
		double speedToleranceMetersPerSecond
	) {
		return isSteerAtTargetPositions(steerPositionTolerance, steerVelocityPerSecondDeadband)
			&& isAtTargetVelocities(speedToleranceMetersPerSecond);
	}

}
