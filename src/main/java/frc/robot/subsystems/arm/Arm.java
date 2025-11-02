package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class Arm extends GBSubsystem {

	private final ControllableMotor arm;
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Rotation2d> velocitySignal;
	private final InputSignal<Rotation2d> positionSignal;
    private final InputSignal<Double> currentSignal;
    private final ArmCommandBuilder armCommandBuilder;
    private final IRequest<Double> armVoltageRequest;
    private final SysIdCalibrator sysIdCalibrator;
	public Arm(
		String logPath,
		ControllableMotor arm,
		InputSignal<Rotation2d> velocitySignal,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Double> voltageSignal,
        InputSignal<Double> currentSignal,
        IRequest<Double> armVoltageRequest,
        ArmCommandBuilder armCommandBuilder,
        SysIdCalibrator sysIdCalibrator
	) {
		super(logPath);
		this.arm = arm;
		this.positionSignal = positionSignal;
		this.velocitySignal = velocitySignal;
		this.voltageSignal = voltageSignal;
        this.currentSignal = currentSignal;
        this.armCommandBuilder = armCommandBuilder;
        this.armVoltageRequest = armVoltageRequest;
        this.sysIdCalibrator = sysIdCalibrator;
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

    public double getCurrent(){
        return currentSignal.getLatestValue();
    }

    public SysIdCalibrator getSysIdCalibrator(){
        return sysIdCalibrator;

    }

    public boolean isAtPosition(Rotation2d targetPosition, Rotation2d tolerance) {
        return positionSignal.isNear(targetPosition, tolerance);
    }

    public boolean isPastPosition(Rotation2d position){
        return positionSignal.isGreater(position);
    }
    public boolean isBehindPosition(Rotation2d position){
        return positionSignal.isLess(position);
    }

    @Override
    protected void subsystemPeriodic() {
        updateInputs();
    }

    private void updateInputs() {
        arm.updateSimulation();
        arm.updateInputs(positionSignal, voltageSignal, velocitySignal);
    }

    public void setVoltage (Double voltage){
        arm.applyRequest(armVoltageRequest.withSetPoint(voltage));
    }
    public void setBrake(boolean brake) {
        arm.setBrake(brake);
    }

//    public void setTargetPosition(Rotation2d targetPosition) {
//        arm.applyRequest(positionRequest.withSetPoint(targetPosition));
//    }

    public void setPower(double power) {
        arm.setPower(power);
    }

}

