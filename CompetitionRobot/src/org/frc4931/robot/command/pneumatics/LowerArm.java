package org.frc4931.robot.command.pneumatics;

import org.frc4931.robot.CompetitionRobot;
import org.frc4931.robot.Subsystems;
import org.frc4931.robot.command.CommandBase;
import org.frc4931.robot.command.SetState;

/**
 * Lowers the roller arm.
 * 
 * @author Zach Anderson
 * @deprecated Use {@link SetState} instead.
 */
public class LowerArm extends CommandBase {

	public LowerArm() {
		requires(Subsystems.rollerArm);
	}

	protected void end() {
		CompetitionRobot.output("Arm Lowered");
	}

	protected void doExecute() {
		Subsystems.rollerArm.arm.lower();
	}

	protected boolean isFinished() {
		return Subsystems.rollerArm.arm.isDown();
	}

}
