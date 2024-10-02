package frc.robot.constants;


import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		20,
		PowerDistribution.ModuleType.kRev
	);

	// ! need to check those IDs which doesn't exists yet since we don't have a working robot

	public static final SparkMaxDeviceID ELEVATOR_FIRST_MOTOR = new SparkMaxDeviceID(2, CANSparkLowLevel.MotorType.kBrushless);

	public static final SparkMaxDeviceID ELEVATOR_SECOND_MOTOR = new SparkMaxDeviceID(3, CANSparkLowLevel.MotorType.kBrushless);

}
