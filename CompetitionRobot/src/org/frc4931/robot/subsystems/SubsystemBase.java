package org.frc4931.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class SubsystemBase extends Subsystem {
	public void putToDashboard() {
		SmartDashboard.putString(getName(), getName());
	}

	protected void initDefaultCommand() {
	}
}
