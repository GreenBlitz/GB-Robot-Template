package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.interfaces.VelocityPositionRequest;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GBSubsystem {

    protected final ControllableMotor motor;
    private final InputSignal<Double> voltageSignal;
    private final InputSignal<Double> currentSignal;
    private final InputSignal<Rotation2d> positionSignal;
    private final InputSignal<Rotation2d> velocitySignal;
    private final VelocityPositionRequest request;
    private final ElevatorPhysicalProperties physicalProperties;
    private double targetPosition;

    public Elevator(
            String logPath,
            ControllableMotor motor,
            InputSignal<Double> voltageSignal,
            InputSignal<Double> currentSignal,
            InputSignal<Rotation2d> positionSignal,
            InputSignal<Rotation2d> velocitySignal,
            VelocityPositionRequest velocityPositionRequest,
            ElevatorPhysicalProperties physicalProperties
    ) {
        super(logPath);
        this.motor = motor;
        this.voltageSignal = voltageSignal;
        this.currentSignal = currentSignal;
        this.positionSignal = positionSignal;
        this.velocitySignal = velocitySignal;
        this.request = velocityPositionRequest;
        //this.commandsBuilder = new RollerCommandsBuilder(this);
        this.motor.resetPosition(Rotation2d.fromRotations(0));
        this.targetPosition = 0;
        this.physicalProperties = physicalProperties;
        //setDefaultCommand(commandsBuilder.stop());
    }

    private Rotation2d getMotorTarget(){
        return physicalProperties.convertFractionalHeightToMotorPosition(targetPosition);
    }

    public double getTargetPositionFractional(){
        return targetPosition;
    }

    public Distance getTargetPositionMeasure(){
        return elevatorHeight.times(targetPosition);
    }

    public void updateTargetPosition(ElevatorMeasure targetPosition) {
        double targetPositionFraction =
        if (!(0<=targetPositionFraction&&targetPositionFraction<=1)){
            System.out.println("Warning: position not in range, called from:");
            Thread.dumpStack();
        }
        this.targetPosition = targetPositionFraction;
    }

    public boolean isAtPosition(double positionFraction, double tolerance){
        return MathUtil.isNear(positionFraction,getElevatorFractionalPosition(),tolerance);
    }
    public boolean isAtPosition(ElevatorMeasure position, ElevatorMeasure tolerance){
        return isAtPosition(position.div(elevatorHeight).magnitude(),tolerance.div(elevatorHeight).magnitude());
    }
    public boolean isAtPosition(double positionFraction, Distance tolerance){
        return isAtPosition(positionFraction, tolerance.div(elevatorHeight).magnitude());
    }

    public void updateTargetPosition(Distance targetPositionHeight){
        updateTargetPosition(targetPositionHeight.div(elevatorHeight).magnitude());
    }

    public Distance getElevatorHeight(){
        return elevatorHeight;
    }

    public void stop() {
        motor.stop();
    }

    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    public Double getVoltage() {
        return voltageSignal.getLatestValue();
    }

    public Double getCurrent() {
        return currentSignal.getLatestValue();
    }

    public Rotation2d getMotorPosition() {
        return positionSignal.getLatestValue();
    }

    public double getElevatorFractionalPosition(){
        return getMotorPosition().minus(motorPositionAtMinHeight).getDegrees()/motorPositionAtMaxHeight.minus(motorPositionAtMinHeight).getDegrees();
    }

    public boolean isAbovePosition(double positionFrac){
        return getElevatorFractionalPosition()>positionFrac;
    }
    public boolean isAbovePosition(Distance position){
        return isAbovePosition(position.div(elevatorHeight).magnitude());
    }
    public boolean isBelowPositon(double positionFrac){
        return getElevatorFractionalPosition()<positionFrac;
    }
    public boolean isBelowPosition(Distance position){
        return isBelowPositon(position.div(elevatorHeight).magnitude());
    }

    public Distance getElevatorPositionMeasure(){
        return elevatorHeight.times(
                getElevatorFractionalPosition()
        );
    }

    private Rotation2d motorPositionOf(double positionFrac){
        return motorPositionAtMaxHeight.times(positionFrac).plus(motorPositionAtMinHeight.times(1-positionFrac));
    }

    public Rotation2d getVelocity() {
        return velocitySignal.getLatestValue();
    }

    public void update() {
        motor.updateSimulation();
        motor.updateInputs(voltageSignal, currentSignal, positionSignal, velocitySignal);
        Logger.recordOutput(getLogPath() + "/PositionTarget", targetPosition);
    }

    public void goToPosition(double positionFrac, double fracHeightPerSecond){
        goToPosition(motorPositionOf(positionFrac),motorPositionAtMaxHeight.minus(motorPositionAtMinHeight).times(fracHeightPerSecond));
    }

    public void goToPosition(Distance position, Distance heightPerSecond){
        goToPosition(position.div(elevatorHeight).magnitude(),heightPerSecond.div(elevatorHeight).magnitude());
    }

    public void goToPosition(double position, Distance heightPerSecond){
        goToPosition(position,heightPerSecond.div(elevatorHeight).magnitude());
    }

    private void goToPosition(Rotation2d position, Rotation2d rps){
        IRequest<Rotation2d> request1 = request.setVelocity(rps).withSetPoint(position);
        motor.applyRequest(request1);
    }

}
