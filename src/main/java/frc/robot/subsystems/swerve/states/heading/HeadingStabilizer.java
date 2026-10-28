package frc.robot.subsystems.swerve.states.heading;

import frc.utils.costomUnits.Angle2d;
import org.wpilib.math.controller.PIDController;
import frc.constants.MathConstants;
import frc.robot.subsystems.swerve.SwerveConstants;

public class HeadingStabilizer {

	private final PIDController headingController;
	private boolean targetLocked;

	public HeadingStabilizer(SwerveConstants constants) {
		this.headingController = new PIDController(
			constants.rotationDegreesPIDController().getP(),
			constants.rotationDegreesPIDController().getI(),
			constants.rotationDegreesPIDController().getD()
		);
		headingController.enableContinuousInput(MathConstants.HALF_CIRCLE.unaryMinus().getDegrees(), MathConstants.HALF_CIRCLE.getDegrees());

		this.targetLocked = false;
	}

	public void lockTarget() {
		targetLocked = true;
	}

	public void unlockTarget() {
		targetLocked = false;
	}

	public void setTargetHeading(Angle2d targetHeading) {
		if (!targetLocked) {
			headingController.reset();
			headingController.setSetpoint(targetHeading.getDegrees());
		}
	}

	public Angle2d calculatePIDOutput(Angle2d currentHeading) {
		return Angle2d.fromDegrees(headingController.calculate(currentHeading.getDegrees()));
	}

}
