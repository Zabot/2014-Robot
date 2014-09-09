package org.frc4931.robot.command.autonomous;

import org.frc4931.robot.Subsystems;
import org.frc4931.robot.command.CommandBase;
import org.frc4931.robot.subsystems.Ranger;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Uses the
 * 
 * @author Zach Anderson
 * 
 */
public class DriveToRange extends CommandBase {
	public static final double SPEED = 0.5;
	private final PIDController pid;
	private final double targetRange;

	/**
	 * Constructs the command with the given sensor and range.
	 * 
	 * @param sensor
	 *            the Sensor to read.
	 * @param range
	 *            the target value.
	 */
	public DriveToRange(Ranger sensor,
			double range) {

		// Instantiate the PIDController using an anonymous class to set the
		// drive speed
		pid = new PIDController(0.1, 0, 0, sensor, new PIDOutput() {
			public void pidWrite(double output) {
				Subsystems.driveTrain.setDriveSpeed(output);
			}
		});

		targetRange = range;

		// Configure the PIDController
		pid.setOutputRange(-SPEED, SPEED);
		pid.setInputRange(0, 250);
		pid.setPercentTolerance(1.0d);

		// Put the PIDController on the dashboard for tuning
		SmartDashboard.putData("Drive PID", pid);
	}

	/**
	 * Sets up and starts the PID Loops.
	 */
	protected void initialize() {
		pid.setSetpoint(targetRange);
		pid.enable();
	}

	protected void doExecute() {

	}

	protected boolean isFinished() {
		return pid.onTarget();
	}

	/**
	 * Closes the PID Loops and stops the drive train.
	 */
	protected void end() {
		pid.disable();
		Subsystems.driveTrain.stop();
	}
}