package org.frc4931.robot.command.roller;

import org.frc4931.robot.Subsystems;
import org.frc4931.robot.command.CommandBase;

/**
 * Rolls the roller out.
 * 
 * @author Zach Anderson
 * 
 */
public class RollOut extends CommandBase {

	public RollOut() {
		requires(Subsystems.rollerArm.roller);
		setInterruptible(true);
	}

	protected void doExecute() {
		Subsystems.rollerArm.roller.rollOut();
	}

	protected boolean isFinished() {
		return false;
	}
}
