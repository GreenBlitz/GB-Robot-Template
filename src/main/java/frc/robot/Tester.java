package frc.robot;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.phoenix6.Phoenix6DoubleSignal;
import frc.robot.hardware.signal.phoenix6.Phoenix6SignalBuilder;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;

public class Tester {

	public static void run() {
		TalonFXWrapper talonFXWrapper = new TalonFXWrapper(0);
		TalonFXMotor phoenixMotor = new TalonFXMotor("Test/Phoenix/", talonFXWrapper, new SysIdRoutine.Config());

		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(new SparkMaxDeviceID(0));
		BrushlessSparkMAXMotor sparkMAXMotor = new BrushlessSparkMAXMotor("Test/Spark/", sparkMaxWrapper, new SysIdRoutine.Config());

		Phoenix6DoubleSignal setPointSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(talonFXWrapper.getClosedLoopReference(), 60);

		SuppliedDoubleSignal hey = new SuppliedDoubleSignal("heyyyyy", sparkMaxWrapper::getVoltage);


		phoenixMotor.updateSignals(setPointSignal, hey);
		sparkMAXMotor.updateSignals(setPointSignal, hey);
	}


}
