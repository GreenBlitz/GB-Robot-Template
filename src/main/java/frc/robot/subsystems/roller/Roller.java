package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Roller extends GBSubsystem {
    private final ControllableMotor roller;
    private final InputSignal<Double> voltageSignal;
    private final InputSignal<Double> currentSignal;

    public Roller(String logPath, ControllableMotor roller, InputSignal<Double> voltageSignal, InputSignal<Double> currentSignal) {
        super(logPath);
        this.roller = roller;
        this.voltageSignal = voltageSignal;
        this.currentSignal = currentSignal;
    }
    public void setVoltage(double voltage){
        roller.setPower(voltage/RollerConstants.MAX_VOLTAGE);
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

    public InputSignal<Rotation2d> getLatencySignal (){
        return latencySignal;
    }

    public InputSignal<Rotation2d> getVelocitySignal (){
        return velocitySignal;
    }

    @Override
    public void subsystemPeriodic(){
        roller.updateSimulation();
        roller.updateInputs(velocitySignal,latencySignal,voltageSignal,currentSignal);
    }
}
