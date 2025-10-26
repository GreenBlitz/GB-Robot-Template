package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Arm extends GBSubsystem {

	private final ControllableMotor arm;
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Rotation2d> velocitySignal;
	private final InputSignal<Rotation2d> positionSignal;
    private final ArmCommandBuilder armCommandBuilder;

	public Arm(
		String logPath,
		ControllableMotor arm,
		InputSignal<Rotation2d> velocitySignal,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Double> voltageSignal,
        ArmCommandBuilder armCommandBuilder
	) {
		super(logPath);
		this.arm = arm;
		this.positionSignal = positionSignal;
		this.velocitySignal = velocitySignal;
		this.voltageSignal = voltageSignal;
        this.armCommandBuilder = armCommandBuilder;
	}

    public ArmCommandBuilder getCommandsBuilder() {
        return armCommandBuilder;
    }

    public Rotation2d getPosition() {
        return positionSignal.getLatestValue();
    }

    public double getVoltage() {
        return voltageSignal.getLatestValue();
    }

    public Rotation2d getVelocity() {
        return velocitySignal.getLatestValue();
    }

    public boolean isAtPosition(Rotation2d targetPosition, Rotation2d tolerance) {
        return positionSignal.isNear(targetPosition, tolerance);
    }

    @Override
    protected void subsystemPeriodic() {
        updateInputs();
    }

    private void updateInputs() {
        arm.updateSimulation();
        arm.updateInputs(positionSignal, voltageSignal, velocitySignal);
    }

    public void setBrake(boolean brake) {
        arm.setBrake(brake);
    }

    public void setTargetPosition(Rotation2d targetPosition) {
        arm.applyRequest(positionRequest.withSetPoint(targetPosition));
    }

    public void setPower(double power) {
        arm.setPower(power);
    }

}
