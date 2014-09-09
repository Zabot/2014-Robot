package org.frc4931.robot.command.pneumatics;

import org.frc4931.robot.CompetitionRobot;
import org.frc4931.robot.Subsystems;
import org.frc4931.robot.command.CommandBase;
import org.frc4931.robot.command.SetState;

/**
 * Raises the roller arm.
 * 
 * @author Zach Anderson
 * @deprecated Use {@link SetState} instead.
 */
public class RaiseArm extends CommandBase {
	public RaiseArm() {
		requires(Subsystems.rollerArm);
	}

	protected void end() {
		CompetitionRobot.output("Arm Raised");
	}

	protected void doExecute() {
		Subsystems.rollerArm.arm.raise();
	}

	protected boolean isFinished() {
		return Subsystems.rollerArm.arm.isUp();
	}

}
