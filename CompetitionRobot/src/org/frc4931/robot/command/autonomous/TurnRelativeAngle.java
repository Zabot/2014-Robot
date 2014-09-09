package org.frc4931.robot.command.autonomous;

import org.frc4931.robot.Subsystems;
import org.frc4931.robot.command.CommandBase;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turns a specified angle using the gyroscope and a pid loop.
 * 
 * @author Zach Anderson
 * 
 */
public class TurnRelativeAngle extends CommandBase {
	public static final double MAX_TURN_SPEED = 0.4;
	public static final double PID_TOLERANCE = 1.0;

	private final PIDController pid;

	private final double angleDelta;
	private double targetAngle;

	/**
	 * Constructs the command.
	 * 
	 * @param angle
	 *            the angle to turn by.
	 */
	public TurnRelativeAngle(double angle) {
		requires(Subsystems.driveTrain);
		requires(Subsystems.imu);

		angleDelta = angle;

		// Instantiate the PIDController using an anonymous class to define the
		// source and set the turn speed
		pid = new PIDController(0.1, 0, 0,
				new PIDSource() {
					public double pidGet() {
						return Subsystems.imu.getBoundedAngle();
					}
				},
				new PIDOutput() {
					public void pidWrite(double output) {
						Subsystems.driveTrain.setTurnSpeed(output);
					}
				});

		// Configure the PIDController
		pid.setOutputRange(-MAX_TURN_SPEED, MAX_TURN_SPEED);
		pid.setPercentTolerance(PID_TOLERANCE);
		pid.setInputRange(0, 360);

		// The PIDController can reach 1 by going up from 359, so it is
		// continuous
		pid.setContinuous(true);

		// Put the PIDController on the dashboard for tuning
		SmartDashboard.putData("Turn PID", pid);
	}

	/**
	 * Sets up and starts the PID loop.
	 */
	protected void initialize() {
		double initialAngle = Subsystems.imu.getContinuousAngle();
		targetAngle = (initialAngle + angleDelta) % 360;

		pid.setSetpoint(targetAngle);
		pid.enable();
	}

	/**
	 * Closes the PID loop and stops the drive train.
	 */
	protected void end() {
		pid.disable();
		Subsystems.driveTrain.stop();
	}

	protected void doExecute() {
		SmartDashboard.putNumber("Angle", Subsystems.imu.getContinuousAngle());
	}

	protected boolean isFinished() {
		return pid.onTarget();
	}
}
