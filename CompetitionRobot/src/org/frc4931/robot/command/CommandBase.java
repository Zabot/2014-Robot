package org.frc4931.robot.command;

import edu.wpi.first.wpilibj.command.Command;

public abstract class CommandBase extends Command {
	private final boolean isContinuous;
	private boolean hasExecuted = false;

	protected CommandBase() {
		this(true);
	}

	protected CommandBase(boolean isContinuous) {
		this.isContinuous = isContinuous;
	}

	protected void initialize() {
		hasExecuted = false;
	}

	protected final void execute() {
		if (isContinuous || (!hasExecuted)) {
			doExecute();
			hasExecuted = true;
		}
	}

	protected abstract void doExecute();

	protected void interrupted() {
		end();
	}

	protected void end() {
		hasExecuted = false;
	}

	protected void requires(Requireable required) {
		super.requires(required.getSubsystem());
	}
}
