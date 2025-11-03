package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.*;
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
    public final IFeedForwardRequest motionMagicRequest;
    private  SysIdCalibrator sysIdCalibrator; //need to do this

	public Arm(
		String logPath,
		ControllableMotor arm,
		InputSignal<Rotation2d> velocitySignal,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Double> voltageSignal,
        InputSignal<Double> currentSignal,
        IRequest<Double> armVoltageRequest,
        IFeedForwardRequest motionMagicRequest
    ) {
		super(logPath);
		this.arm = arm;
		this.positionSignal = positionSignal;
		this.velocitySignal = velocitySignal;
		this.voltageSignal = voltageSignal;
        this.currentSignal = currentSignal;
        this.armVoltageRequest = armVoltageRequest;
        this.motionMagicRequest = motionMagicRequest;
        armCommandBuilder = new ArmCommandBuilder(this);
        setDefaultCommand(armCommandBuilder.stayInPlace());
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

    public SysIdCalibrator getSysIdCalibrator() {
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
        arm.updateSimulation();
        updateInputs();
    }

    private void updateInputs() {
        arm.updateInputs(positionSignal, voltageSignal, velocitySignal,currentSignal);
    }

    public void log(){

    }

    public void setVoltage (Double voltage){
        arm.applyRequest(armVoltageRequest.withSetPoint(voltage));
    }
    public void setBrake(boolean brake) {
        arm.setBrake(brake);
    }

    public void setTargetPosition(Rotation2d targetPosition) {
        arm.applyRequest(motionMagicRequest.withSetPoint(targetPosition));

    }

    public void setPower(double power) {
        arm.setPower(power);
    }

    protected void stayInPlace(){
        setTargetPosition(positionSignal.getLatestValue());
    }


}

