package frc.robot.subsystems.funnel;

public enum FunnelState {

    INTAKE(0.3),
    OUTTAKE(-0.3),
    DEFAULT(0);

    private final double power;

    FunnelState(double power){
        this.power = power;
    }

    public double getPower(){
        return power;
    }

}
