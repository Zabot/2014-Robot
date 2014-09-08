package org.frc4931.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

public abstract class SubsystemBase extends Subsystem {
	public abstract void putToDashboard();

	protected void initDefaultCommand() {
	}
}
