package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Roller extends GBSubsystem {
    private final ControllableMotor roller;
    private final InputSignal<Double> voltageSignal;
    private final InputSignal<Double> currentSignal;
    private final InputSignal<Rotation2d> positionSignal;
    private final RollerCommandsBuilder commandsBuilder = new RollerCommandsBuilder(this);
    private final IRequest<Double> setVoltageRequest;
    private final IRequest<Rotation2d> goToPositionRequest;
    private final Rotation2d tolerance;

    public Roller(String logPath, ControllableMotor roller, InputSignal<Double> voltageSignal, InputSignal<Double> currentSignal, InputSignal<Rotation2d> positionSignal, IRequest<Double> setVoltageRequest, IRequest<Rotation2d> goToPositionRequest, Rotation2d tolerance) {
        super(logPath);
        this.roller = roller;
        this.voltageSignal = voltageSignal;
        this.currentSignal = currentSignal;
        this.positionSignal = positionSignal;
        this.setVoltageRequest = setVoltageRequest;
        this.goToPositionRequest = goToPositionRequest;
        this.tolerance = tolerance;
    }
    public void setVoltage(double voltage){
        roller.applyRequest(setVoltageRequest.withSetPoint(voltage));
    }

    public void setPower(double power){
        roller.setPower(power);
    }

    public void stop(){
        roller.stop();
    }

    public void setBrake(boolean brake){
        roller.setBrake(brake);
    }

    public InputSignal<Double> getVoltageSignal (){
        return voltageSignal;
    }

    public InputSignal<Double> getCurrentSignal (){
        return currentSignal;
    }

    public InputSignal<Rotation2d> getPositionSignal(){
        return positionSignal;
    }

    public RollerCommandsBuilder getCommands(){
        return commandsBuilder;
    }
    public boolean isAtPosition(Rotation2d position){
        return (positionSignal.isNear(position,tolerance));
    }
    public boolean isPastPosition(Rotation2d position){
        if (!isAtPosition(position)){
            return (position.getDegrees() > positionSignal.getLatestValue().getDegrees());
        }
        return false;
    }

    public void goToPosition(Rotation2d position){
        roller.applyRequest(goToPositionRequest.withSetPoint(position));
    }

    @Override
    public void subsystemPeriodic(){
        roller.updateSimulation();
        roller.updateInputs(voltageSignal,currentSignal, positionSignal);
    }
}
