// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtil;
import frc.utils.time.TimeUtil;
import frc.utils.logger.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;
import frc.utils.brakestate.BrakeStateManager;
import org.littletonrobotics.junction.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private final Robot robot;
	private Command autonomousCommand;
	private int roborioCycles;

	public RobotManager() {
		LoggerFactory.initializeLogger();
		PathPlannerUtil.startPathfinder();

		this.roborioCycles = 0;
		this.robot = new Robot();

		JoysticksBindings.configureBindings(robot);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.coast();
		}
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.brake();
		}
	}

	@Override
	public void autonomousInit() {
		this.autonomousCommand = robot.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		JoysticksBindings.setDriversInputsToSwerve(robot.getSwerve());
		robot.periodic();
		AlertManager.reportAlerts();
//		Logger.recordOutput("Slots", Field.CORAL_STATION_SLOTS_MIDDLES);
//		Logger.recordOutput("Slots2",Field.CORAL_STATION_SLOTS_MIDDLES[CoralStationSlot.R2.getIndex()]);
//		Logger.recordOutput("Slots3",Field.CORAL_STATION_SLOTS_MIDDLES[CoralStationSlot.R3.getIndex()]);
//		Logger.recordOutput("Slotsssssss", new Pose2d[]{
//				Field.getCoralStationSlotsPose2d(CoralStationSlot.R1), Field.getCoralStationSlotsPose2d(CoralStationSlot.R2),
//				Field.getCoralStationSlotsPose2d(CoralStationSlot.R3), Field.getCoralStationSlotsPose2d(CoralStationSlot.R4),
//				Field.getCoralStationSlotsPose2d(CoralStationSlot.R5), Field.getCoralStationSlotsPose2d(CoralStationSlot.R6),
//				Field.getCoralStationSlotsPose2d(CoralStationSlot.R7), Field.getCoralStationSlotsPose2d(CoralStationSlot.R8),
//				Field.getCoralStationSlotsPose2d(CoralStationSlot.R9),
//
//				Field.getCoralStationSlotsPose2d(CoralStationSlot.L1), Field.getCoralStationSlotsPose2d(CoralStationSlot.L2),
//				Field.getCoralStationSlotsPose2d(CoralStationSlot.L3), Field.getCoralStationSlotsPose2d(CoralStationSlot.L4),
//				Field.getCoralStationSlotsPose2d(CoralStationSlot.L5), Field.getCoralStationSlotsPose2d(CoralStationSlot.L6),
//				Field.getCoralStationSlotsPose2d(CoralStationSlot.L7), Field.getCoralStationSlotsPose2d(CoralStationSlot.L8),
//				Field.getCoralStationSlotsPose2d(CoralStationSlot.L9)
//		});
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtil.updateCycleTime(roborioCycles);
	}

}
