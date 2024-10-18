// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.IDs;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.factories.ElevatorFactory;
import frc.robot.subsystems.elevatorRoller.ElevatorRollerConstants;
import frc.robot.subsystems.elevatorRoller.factory.ElevatorRollerFactory;
import frc.robot.subsystems.flywheel.factory.FlywheelFactory;
import frc.robot.subsystems.intake.pivot.Pivot;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.elevatorRoller.ElevatorRoller;
import frc.robot.subsystems.intake.pivot.PivotConstants;
import frc.robot.subsystems.intake.pivot.factory.PivotFactory;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRollerConstant;
import frc.robot.subsystems.intake.roller.factory.IntakeRollerFactory;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.funnel.factory.FunnelFactory;
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
	private final ElevatorRoller elevatorRoller;
	private final Funnel funnel;
	private final Pivot pivot;
	private final IntakeRoller intakeRoller;
	private final Elevator elevator;
	private final Flywheel flywheel;

	private final Superstructure superstructure;

	public Robot() {
		this.swerve = null;// new Swerve(
//			SwerveConstantsFactory.create(SwerveType.SWERVE),
//			ModulesFactory.create(SwerveType.SWERVE),
//			GyroFactory.create(SwerveType.SWERVE)
//		);
		this.elevatorRoller = new ElevatorRoller(ElevatorRollerFactory.create(ElevatorRollerConstants.LOG_PATH));
		this.funnel = new Funnel(FunnelFactory.create(FunnelConstants.LOG_PATH));
		this.pivot = new Pivot(PivotFactory.create(PivotConstants.LOG_PATH));
		BrakeStateManager.add(() -> pivot.setBrake(true), () -> pivot.setBrake(false));
		this.intakeRoller = new IntakeRoller(IntakeRollerFactory.create(IntakeRollerConstant.LOG_PATH));
		this.elevator = new Elevator(ElevatorFactory.create(ElevatorConstants.LOG_PATH));
		BrakeStateManager.add(() -> elevator.setBrake(true), () -> elevator.setBrake(false));
		this.flywheel = FlywheelFactory
			.create("TopMotor/", "BottomMotor/", IDs.CANSparkMAXIDs.TOP_FLYWHEEL, IDs.CANSparkMAXIDs.BOTTOM_FLYWHEEL, this);
		this.superstructure = new Superstructure("Superstructure/", this);

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

	public ElevatorRoller getElevatorRoller() {
		return elevatorRoller;
	}

	public Funnel getFunnel() {
		return funnel;
	}

	public Pivot getPivot() {
		return pivot;
	}

	public Elevator getElevator() {
		return elevator;
	}

	public Flywheel getFlywheel() {
		return flywheel;
	}

	public IntakeRoller getIntakeRoller() {
		return intakeRoller;
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

}
