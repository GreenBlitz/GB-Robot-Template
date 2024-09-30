// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.funnel.factory.FunnelFactory;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.factory.IntakeFactory;
import frc.robot.subsystems.elbow.Elbow;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.elbow.factory.ElbowFactory;
import frc.robot.subsystems.flywheel.FlyWheelConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.factory.FlywheelFactory;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.factory.PivotFactory;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.factory.RollerFactory;
import frc.robot.subsystems.solenoid.Solenoid;
import frc.robot.subsystems.solenoid.SolenoidConstants;
import frc.robot.subsystems.solenoid.factory.SolenoidFactory;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.superstructure.Superstructure;
import frc.utils.brakestate.BrakeStateManager;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final Solenoid solenoid;
	private final Funnel funnel;
	private final Intake intake;
	private final Elbow elbow;
	private final Flywheel flywheel;
	private final Pivot pivot;
	private final Roller roller;

	private final Superstructure superstructure;

	public Robot() {
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveName.SWERVE),
			ModulesFactory.create(SwerveName.SWERVE),
			GyroFactory.create(SwerveName.SWERVE)
		);
		this.solenoid = new Solenoid(SolenoidFactory.create(SolenoidConstants.LOG_PATH));
		this.intake = new Intake(IntakeFactory.create(IntakeConstants.LOG_PATH));
		this.flywheel = new Flywheel(FlywheelFactory.create(FlyWheelConstants.LOG_PATH));
		this.pivot = new Pivot(PivotFactory.create(PivotConstants.LOG_PATH));
		BrakeStateManager.add(() -> pivot.setBrake(true), () -> pivot.setBrake(false));
		this.elbow = new Elbow(ElbowFactory.create(ElbowConstants.LOG_PATH));
		BrakeStateManager.add(() -> elbow.setBrake(true), () -> elbow.setBrake(false));
		this.funnel = new Funnel(FunnelFactory.create(FunnelConstants.LOG_PATH));
		this.roller = new Roller(RollerFactory.create(RollerConstants.LOG_PATH));
		BrakeStateManager.add(() -> roller.setBrake(true), () -> roller.setBrake(false));

		this.superstructure = new Superstructure(this);

		configureBindings();
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}


	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public Solenoid getSolenoid() {
		return solenoid;
	}

	public Funnel getFunnel() {
		return funnel;
	}

	public Intake getIntake() {
		return intake;
	}

	public Elbow getElbow() {
		return elbow;
	}

	public Flywheel getFlywheel() {
		return flywheel;
	}

	public Pivot getPivot() {
		return pivot;
	}

	public Roller getRoller() {
		return roller;
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}


}
