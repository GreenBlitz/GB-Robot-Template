package frc.utils.mirrorutils;

import frc.robot.constants.FieldConstants;
import frc.utils.DriverStationUtils;

/**
 * A class that allows for objects to be mirrored across the center of the field when the robot is on the red alliance.
 * This is useful for placing field elements and other objects that are mirrored across the field, or for mirroring the target
 * heading to face a field element.
 *
 * @param <T> the type of object to mirror
 */
public abstract class Mirrorable<T> {

    protected final T nonMirroredObject, mirroredObject;

    protected final boolean mirrorWhenNotOnRelativeAlliance;


    protected Mirrorable(T nonMirroredObject, boolean mirrorWhenNotOnRelativeAlliance) {
        this.nonMirroredObject = nonMirroredObject;
        this.mirroredObject = mirror(nonMirroredObject);
        this.mirrorWhenNotOnRelativeAlliance = mirrorWhenNotOnRelativeAlliance;
    }

    public T get() {
        return DriverStationUtils.getAlliance() == FieldConstants.RELATIVE_FIELD_CONVETION_ALLIANCE && mirrorWhenNotOnRelativeAlliance
                ? mirroredObject
                : nonMirroredObject;
    }


    protected abstract T mirror(T object);

}
