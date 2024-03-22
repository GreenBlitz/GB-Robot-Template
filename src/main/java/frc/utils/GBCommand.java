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
 */
public abstract class GBCommand extends Command {
	
	protected Set<Subsystem> systems;
	
	/**
	 * Constructor for a command that doesn't use a subsystem at all
	 */
	public GBCommand() {
		this.systems = new HashSet<>();
	}
	
	/**
	 * @param systems - The subsystems used by this command
	 */
	public GBCommand(Subsystem... systems) {
		this.systems = new HashSet<>();
		this.systems.addAll(Arrays.asList(systems));
	}
	
	/**
	 * adds a new subsystem requirement
	 *
	 * @param sys - the new subsystem
	 */
	public void require(Subsystem sys) {
		this.systems.add(sys);
	}
	
	/**
	 * Checks if a subsystem is used by a command
	 *
	 * @param sys - the subsystem to check for
	 * @return if the subsystem is used
	 */
	public boolean isRequired(Subsystem sys) {
		return systems.contains(sys);
	}
	
	@Override
	/**
	 * @return a set of the required subsystems
	 */
	public Set<Subsystem> getRequirements() {
		return systems;
	}
}
