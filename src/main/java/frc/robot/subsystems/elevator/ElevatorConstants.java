package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorConstants {

	public static final String LOG_PATH = "Subsystems/Elevator/";

	public static final int ELEVATOR_PID_SLOT = 0;

	public static ElevatorFeedforward FEEDFORwARD_CALCULATOR = new ElevatorFeedforward(0, 0, 0, 0);

}
