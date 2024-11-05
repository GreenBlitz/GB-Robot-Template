package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class Modules extends GBSubsystem {

	private final Module[] modules;
	private final ModulesCommandsBuilder commandsBuilder;

	public Modules(String logPath, Module... modules) {
		super(logPath + ModuleConstants.LOG_PATH_ADDITION);
		this.modules = modules;
		this.commandsBuilder = new ModulesCommandsBuilder(this);
		updateInputs();
	}

	public Module getModule(ModuleUtils.ModulePosition modulePosition) {
		return modules[modulePosition.getIndex()];
	}

	public ModulesCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void updateInputs() {
		for (Module currentModule : modules) {
			currentModule.updateStatus();
		}
		Logger.recordOutput(getLogPath() + "CurrentStates", getCurrentStates());
		Logger.recordOutput(getLogPath() + "TargetStates", getTargetStates());
	}


	public void resetModulesAngleByEncoder() {
		for (Module module : modules) {
			module.resetByEncoder();
		}
	}

	public void setBrake(boolean brake) {
		for (Module currentModule : modules) {
			currentModule.setBrake(brake);
		}
	}


	protected void pointWheels(Rotation2d targetSteerPosition, boolean optimize) {
		for (Module module : modules) {
			module.pointSteer(targetSteerPosition, optimize);
		}
	}

	protected void pointWheelsInCircle() {
		boolean optimizeAngle = true;
		modules[ModuleUtils.ModulePosition.FRONT_LEFT.getIndex()].pointSteer(MathConstants.EIGHTH_CIRCLE.unaryMinus(), optimizeAngle);
		modules[ModuleUtils.ModulePosition.FRONT_RIGHT.getIndex()].pointSteer(MathConstants.EIGHTH_CIRCLE, optimizeAngle);
		modules[ModuleUtils.ModulePosition.BACK_LEFT.getIndex()].pointSteer(MathConstants.EIGHTH_CIRCLE, optimizeAngle);
		modules[ModuleUtils.ModulePosition.BACK_RIGHT.getIndex()].pointSteer(MathConstants.EIGHTH_CIRCLE.unaryMinus(), optimizeAngle);
	}

	public void pointWheelsInX(boolean isClosedLoop) {
		SwerveModuleState frontLeftBackRight = new SwerveModuleState(0, MathConstants.EIGHTH_CIRCLE);
		SwerveModuleState frontRightBackLeft = new SwerveModuleState(0, MathConstants.EIGHTH_CIRCLE.unaryMinus());

		modules[ModuleUtils.ModulePosition.FRONT_LEFT.getIndex()].setTargetState(frontLeftBackRight, isClosedLoop);
		modules[ModuleUtils.ModulePosition.FRONT_RIGHT.getIndex()].setTargetState(frontRightBackLeft, isClosedLoop);
		modules[ModuleUtils.ModulePosition.BACK_LEFT.getIndex()].setTargetState(frontRightBackLeft, isClosedLoop);
		modules[ModuleUtils.ModulePosition.BACK_RIGHT.getIndex()].setTargetState(frontLeftBackRight, isClosedLoop);
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

	public void setDrivesVoltage(double voltage) {
		for (Module module : modules) {
			module.setDriveVoltage(voltage);
		}
	}

	public void setTargetStates(SwerveModuleState[] moduleStates, boolean isClosedLoop) {
		for (int i = 0; i < modules.length; i++) {
			modules[i].setTargetState(moduleStates[i], isClosedLoop);
		}
	}


	public int getNumberOfOdometrySamples() {
		int numberOfOdometrySamples = modules[0].getNumberOfOdometrySamples();
		for (int i = 1; i < modules.length; i++) {
			numberOfOdometrySamples = Math.min(numberOfOdometrySamples, modules[i].getNumberOfOdometrySamples());
		}
		return numberOfOdometrySamples;
	}

	public SwerveModuleState[] getTargetStates() {
		return Arrays.stream(modules).map(Module::getTargetState).toArray(SwerveModuleState[]::new);
	}

	public SwerveModuleState[] getCurrentStates() {
		return Arrays.stream(modules).map(Module::getCurrentState).toArray(SwerveModuleState[]::new);
	}

	public Rotation2d[] getDrivesAngles() {
		return Arrays.stream(modules).map(Module::getDriveAngle).toArray(Rotation2d[]::new);
	}

	public SwerveModulePosition[] getWheelsPositions(int odometrySampleIndex) {
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

	public boolean isSteersAtTargetPositions(Rotation2d steerTolerance, Rotation2d steerVelocityPerSecondDeadband) {
		for (Module module : modules) {
			if (!module.isSteerAtTargetPosition(steerTolerance, steerVelocityPerSecondDeadband)) {
				return false;
			}
		}
		return true;
	}

	public boolean isAtTargetStates(Rotation2d steerTolerance, Rotation2d steerVelocityPerSecondDeadband, double speedToleranceMetersPerSecond) {
		return isSteersAtTargetPositions(steerTolerance, steerVelocityPerSecondDeadband) && isAtTargetVelocities(speedToleranceMetersPerSecond);
	}

}
