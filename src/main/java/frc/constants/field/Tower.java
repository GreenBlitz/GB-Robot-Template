package frc.constants.field;

import edu.wpi.first.math.geometry.Translation2d;

public enum Tower {

    CLOSE_TOWER(new Translation2d(0,-1)),
    FAR_TOWER(new Translation2d(0,1)),
    RIGHT_TOWER(new Translation2d(1,0)),
    LEFT_TOWER(new Translation2d(-1,0));

    private final Translation2d tower;

    Tower(Translation2d tower){
        this.tower = tower;
    }

    public Translation2d getTower() {
        return tower;
    }
}
