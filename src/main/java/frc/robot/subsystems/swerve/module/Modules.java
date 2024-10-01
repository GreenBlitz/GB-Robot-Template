package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.MathConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class Modules {

	private final Module[] modules;
	private final String logPath;

	public Modules(String logPath, Module[] modules) {
		this.modules = modules;
		this.logPath = logPath + ModuleConstants.LOG_PATH_ADDITION;
	}

	public Module getModule(ModuleUtils.ModulePosition modulePosition) {
		return modules[modulePosition.getIndex()];
	}

	public void logStatus() {
		for (Module currentModule : modules) {
			currentModule.updateInputs();
		}
		Logger.recordOutput(logPath + "CurrentStates", getCurrentStates());
		Logger.recordOutput(logPath + "TargetStates", getTargetStates());
	}


	public void pointWheels(Rotation2d targetAngle, boolean optimize) {
		for (Module module : modules) {
			module.pointToAngle(targetAngle, optimize);
		}
	}

	public void pointWheelsInCircle() {
		boolean optimizeAngle = true;
		modules[0].pointToAngle(MathConstants.EIGHTH_CIRCLE.unaryMinus(), optimizeAngle);
		modules[1].pointToAngle(MathConstants.EIGHTH_CIRCLE, optimizeAngle);
		modules[2].pointToAngle(MathConstants.EIGHTH_CIRCLE, optimizeAngle);
		modules[3].pointToAngle(MathConstants.EIGHTH_CIRCLE.unaryMinus(), optimizeAngle);
	}

	public void pointWheelsInX(boolean isClosedLoop) {
		SwerveModuleState frontLeftBackRight = new SwerveModuleState(0, MathConstants.EIGHTH_CIRCLE);
		SwerveModuleState frontRightBackLeft = new SwerveModuleState(0, MathConstants.EIGHTH_CIRCLE.unaryMinus());

		modules[0].setTargetState(frontLeftBackRight, isClosedLoop);
		modules[1].setTargetState(frontRightBackLeft, isClosedLoop);
		modules[2].setTargetState(frontRightBackLeft, isClosedLoop);
		modules[3].setTargetState(frontLeftBackRight, isClosedLoop);
	}


	public void setSteersVoltage(ModuleUtils.ModulePosition module, double voltage) {
		modules[module.getIndex()].setSteerVoltage(voltage);
	}

	public void setDrivesVoltage(double voltage) {
		for (Module module : modules) {
			module.setDriveVoltage(voltage);
		}
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

	public void setTargetStates(SwerveModuleState[] moduleStates, boolean isClosedLoop) {
		for (int i = 0; i < modules.length; i++) {
			modules[i].setTargetState(moduleStates[i], isClosedLoop);
		}
	}

	public void stop() {
		for (Module currentModule : modules) {
			currentModule.stop();
		}
	}


	public boolean isAtTargetVelocities() {
		for (Module module : modules) {
			if (!module.isAtTargetVelocity()) {
				return false;
			}
		}
		return true;
	}

	public boolean isAtTargetAngles() {
		for (Module module : modules) {
			if (!module.isAtTargetAngle()) {
				return false;
			}
		}
		return true;
	}

	public boolean isAtTargetStates() {
		return isAtTargetAngles() && isAtTargetAngles();
	}


	public SwerveModuleState[] getTargetStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];

		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getTargetState();
		}

		return states;
	}

	public SwerveModuleState[] getCurrentStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];

		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getCurrentState();
		}

		return states;
	}


	public Rotation2d[] getDrivesAngles() {
		return Arrays.stream(modules).map(Module::getDriveAngle).toArray(Rotation2d[]::new);
	}

	public int getNumberOfOdometrySamples() {
		int numberOfOdometrySamples = modules[0].getNumberOfOdometrySamples();
		for (int i = 1; i < modules.length; i++) {
			numberOfOdometrySamples = Math.min(numberOfOdometrySamples, modules[i].getNumberOfOdometrySamples());
		}
		return numberOfOdometrySamples;
	}

	public SwerveDriveWheelPositions getWheelsPositions(int odometrySampleIndex) {
		SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			swerveModulePositions[i] = modules[i].getOdometryPosition(odometrySampleIndex);
		}
		return new SwerveDriveWheelPositions(swerveModulePositions);
	}

}
