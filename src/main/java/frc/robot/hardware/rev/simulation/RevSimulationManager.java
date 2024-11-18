package frc.robot.hardware.rev.simulation;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;

public class RevSimulationManager {

	private static final REVPhysicsSim revPhysicsSim = new REVPhysicsSim();

	public static void addSparkMax(SparkMaxWrapper sparkMaxWrapper) {
		revPhysicsSim.addSparkMax(sparkMaxWrapper, DCMotor.getNEO(1));
	}

	public static void addSparkMax(SparkMaxWrapper sparkMax, DCMotor dcMotor) {
		revPhysicsSim.addSparkMax(sparkMax, dcMotor);
	}

	public static void addSparkMax(SparkMaxWrapper sparkMax, final float stallTorque, final float maxFreeSpeed) {
		revPhysicsSim.addSparkMax(sparkMax, stallTorque, maxFreeSpeed);
	}

	public static void run() {
		revPhysicsSim.run();
	}

}
