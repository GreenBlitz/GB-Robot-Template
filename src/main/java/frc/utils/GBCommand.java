package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * This class adds the ability to track the subsystems used for a given command
 *
 * @author Nitzan
 * @author Raz
 * @author Tal -ish
 * @author Noam Rosenberg
 * @author Yoav Herman
 * @author Yoni Kiriaty
 */
public abstract class GBCommand extends Command {

    protected Set<Subsystem> subsystems;

    /**
     * Constructor for a command that doesn't use a subsystem at all
     */
    public GBCommand() {
        this.subsystems = new HashSet<>();
    }

    /**
     * @param subsystems - The subsystems used by this command
     */
    public GBCommand(Subsystem... subsystems) {
        this.subsystems = new HashSet<>();
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    /**
     * adds a new subsystem requirement
     *
     * @param subsystem - the new subsystem
     */
    public void require(Subsystem subsystem) {
        this.subsystems.add(subsystem);
    }

    public void require(Subsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    /**
     * Checks if a subsystem is used by a command
     *
     * @param subsystem - the subsystem to check for
     * @return if the subsystem is used
     */
    public boolean isRequired(Subsystem subsystem) {
        return subsystems.contains(subsystem);
    }

    /**
     * @return a set of the required subsystems
     */
    @Override
    public Set<Subsystem> getRequirements() {
        return subsystems;
    }

}
