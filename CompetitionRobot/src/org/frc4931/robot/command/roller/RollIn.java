package org.frc4931.robot.command.roller;

import org.frc4931.robot.Subsystems;
import org.frc4931.robot.command.CommandBase;

/**
 * Rolls the roller in.
 * 
 * @author Zach Anderson
 * 
 */
public class RollIn extends CommandBase {

	public RollIn() {
		requires(Subsystems.rollerArm.roller);
		setInterruptible(true);
	}

	protected void doExecute() {
		Subsystems.rollerArm.roller.rollIn();
	}

	protected boolean isFinished() {
		return false;
	}
}
