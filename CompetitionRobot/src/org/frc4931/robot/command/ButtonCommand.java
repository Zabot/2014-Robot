package org.frc4931.robot.command;

import edu.wpi.first.wpilibj.command.Scheduler;

public class ButtonCommand extends OneShotCommand{
	private final CommandBase template;
	
	public ButtonCommand(CommandBase command) {
		template = command;
	}

	protected void doExecute() {
		Scheduler.getInstance().add((CommandBase)template.copy());
	}
	

}
