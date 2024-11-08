package frc.robot.subsystems.swerve.module.maple;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public record MapleModuleConstants(
	PIDController steerPIDControllerRadians,
	PIDController drivePIDControllerRadiansPerSecond,
	SimpleMotorFeedforward driveFeedForwardRadiansPerSecond
) {

	public MapleModuleConstants(
		PIDController steerPIDControllerRadians,
		PIDController drivePIDControllerRadiansPerSecond,
		SimpleMotorFeedforward driveFeedForwardRadiansPerSecond
	) {
		this.steerPIDControllerRadians = steerPIDControllerRadians;
		this.drivePIDControllerRadiansPerSecond = drivePIDControllerRadiansPerSecond;
		this.driveFeedForwardRadiansPerSecond = driveFeedForwardRadiansPerSecond;

		this.steerPIDControllerRadians.enableContinuousInput(-Math.PI, Math.PI);
	}

	public MapleModuleConstants(PIDConstants steerPIDConstants, PIDConstants drivePIDConstants, SimpleMotorFeedforward driveFeedForward) {
		this(
			new PIDController(steerPIDConstants.kP, steerPIDConstants.kI, steerPIDConstants.kD),
			new PIDController(drivePIDConstants.kP, drivePIDConstants.kI, drivePIDConstants.kD),
			driveFeedForward
		);
	}

}
