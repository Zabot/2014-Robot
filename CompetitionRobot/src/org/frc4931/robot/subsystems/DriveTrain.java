package org.frc4931.robot.subsystems;

import org.frc4931.zach.drive.AbstractMotor.SpeedControllerType;
import org.frc4931.zach.drive.ContinuousMotor;
import org.frc4931.zach.utils.Transform;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Wrapper for {@link RobotDrive}. Also provides increased functionality in the
 * form of inertial compensation and maximum acceleration.
 * 
 * @author Zach Anderson
 */
public class DriveTrain extends SubsystemBase {
	/** The {@link RobotDrive} being wrapped. */
	private final RobotDrive drive;

	/** The target drive speed. */
	private double driveSpeed = 0;
	/** The target turn speed. */
	private double turnSpeed = 0;

	/**
	 * Constructs a four motor {@link DriveTrain} using motors of the specified
	 * {@link SpeedControllerType} on the specified channels.
	 * 
	 * @param leftFrontMotor
	 *            The channel of the left front motor.
	 * @param leftRearMotor
	 *            The channel of the left rear motor.
	 * @param rightFrontMotor
	 *            The channel of the right front motor.
	 * @param rightRearMotor
	 *            The channel of the right rear motor.
	 * @param controller
	 *            The type of speed controller.
	 */
	public DriveTrain(
			int leftFrontMotor,
			int leftRearMotor,
			int rightFrontMotor,
			int rightRearMotor,
			ContinuousMotor.SpeedControllerType type) {
		this(new ContinuousMotor(leftFrontMotor, type),
				new ContinuousMotor(leftRearMotor, type),
				new ContinuousMotor(rightFrontMotor, type),
				new ContinuousMotor(rightRearMotor, type));
	}

	/**
	 * Constructs a four motor {@link DriveTrain} using the specified already
	 * instantiated {@link ContinuousMotor}s.
	 * 
	 * @param leftFrontMotor
	 *            The front left motor
	 * @param leftRearMotor
	 *            The back left motor
	 * @param rightFrontMotor
	 *            The front right motor
	 * @param rightRearMotor
	 *            The back right motor.
	 */
	public DriveTrain(
			ContinuousMotor leftFrontMotor,
			ContinuousMotor leftRearMotor,
			ContinuousMotor rightFrontMotor,
			ContinuousMotor rightRearMotor) {
		drive = new RobotDrive(leftFrontMotor.getController(),
				leftRearMotor.getController(),
				rightFrontMotor.getController(),
				rightRearMotor.getController());
	}

	/**
	 * Constructs a two motor {@link DriveTrain} using motors of the specified
	 * {@link SpeedControllerType} on the specified channels.
	 * 
	 * @param driveMotorLeft
	 *            The channel of the left motor.
	 * @param driveMotorRight
	 *            The channel of the right motor.
	 * @param type
	 *            The type of the speed controller.
	 */
	public DriveTrain(
			int driveMotorLeft,
			int driveMotorRight,
			ContinuousMotor.SpeedControllerType type) {
		this(new ContinuousMotor(driveMotorLeft, type),
				new ContinuousMotor(driveMotorRight, type));
	}

	/**
	 * Constructs a two motor {@link DriveTrain} using the specified already
	 * instantiated {@link ContinuousMotor}s.
	 * 
	 * @param leftFrontMotor
	 *            The front left motor
	 * @param leftRearMotor
	 *            The back left motor
	 * @param rightFrontMotor
	 *            The front right motor
	 * @param rightRearMotor
	 *            The back right motor.
	 */
	public DriveTrain(
			ContinuousMotor leftMotor,
			ContinuousMotor rightMotor) {
		drive = new RobotDrive(leftMotor.getController(),
				rightMotor.getController());
	}

	/**
	 * Drive this {@link DriveTrain} in tank mode.
	 * 
	 * @param leftSpeed
	 *            The speed of the left motor from -1.0 to 1.0.
	 * @param rightSpeed
	 *            The speed of the right motor from -1.0 to 1.0.
	 * @deprecated Use setDriveSpeed() and setTurnSpeed() instead.
	 */
	public void tankDrive(double leftSpeed, double rightSpeed) {
		drive.tankDrive(leftSpeed, rightSpeed);
	}

	/**
	 * Drive this {@link DriveTrain} in arcade mode.
	 * 
	 * @param drive
	 *            The forward speed from -1.0 to 1.0.
	 * @param turn
	 *            The turn rate from -1.0 to 1.0.
	 */
	public void arcadeDrive(double drive, double turn) {
		// Find the delta
		double delta = drive - driveSpeed;

		// Get the sign of delta
		int deltaSign = Transform.signum(delta);

		// Disregard the sign from delta
		delta = Math.abs(delta);

		// Clamp delta to the max allowed drive speed
		delta = Math.min(delta, getMaxDelta(driveSpeed));

		// Update speeds
		setDriveSpeed(driveSpeed + (delta * deltaSign));
		setTurnSpeed(turn);
	}

	/**
	 * Finds the maximum amount the drive speed of the robot can change in one
	 * iteration given an initial speed.
	 * 
	 * @param speed
	 *            The speed to find the max delta for
	 * @return The maximum amount the speed can change in one iteration.
	 */
	private static final double getMaxDelta(double speed) {
		// TODO Make constants, maybe in an AccelerationCurve class?

		// Disregard the sign of speed
		speed = Math.abs(speed);

		if (speed < SmartDashboard.getNumber("Range 1")) {
			return SmartDashboard.getNumber("Max Delta 1");
		} else if (speed < SmartDashboard.getNumber("Range 2")) {
			return SmartDashboard.getNumber("Max Delta 2");
		} else if (speed <= SmartDashboard.getNumber("Range 3")) {
			return SmartDashboard.getNumber("Max Delta 3");
		} else {
			return 0.0d;
		}
	}

	/**
	 * Update the speed of the motors in this {@link DriveTrain}. <b>This method
	 * must be called once each iteration or the watchdog will stop the motors
	 * and display an error, "Output not updated often enough".</b>
	 */
	public void update() {
		// Get the sign of driveSpeed
		int driveDirection = Transform.signum(driveSpeed);

		// TODO Make this a constant
		// Get the minimum drive speed to overcome inertia
		double minDriveSpeed = SmartDashboard.getNumber("MinDriveSpeed");

		// TODO Make this a constant
		// Get the maximum allowed drive speed
		double maxDriveSpeed = SmartDashboard.getNumber("MaxDriveSpeed");

		// Map the unsigned speed to that range
		double mappedSpeed = Transform.map(0, 1, minDriveSpeed,
				maxDriveSpeed, Math.abs(driveSpeed));

		// Restore the sign
		mappedSpeed = mappedSpeed * driveDirection;

		// Get the sign of turnSpeed
		int turnDirection = Transform.signum(turnSpeed);

		// TODO Make this a constant
		// Get the minimum drive speed to overcome inertia
		double minTurnSpeed = SmartDashboard.getNumber("MinTurnSpeed");

		// TODO Make this a constant
		// Get the maximum allowed drive speed
		double maxTurnSpeed = SmartDashboard.getNumber("MaxTurnSpeed");

		// Map the unsigned speed to that range
		double mappedTurn = Transform.map(0, 1, minTurnSpeed,
				maxTurnSpeed, Math.abs(turnSpeed));

		// Restore the sign
		mappedTurn = mappedTurn * turnDirection;

		// Update the motors with the new speeds
		drive.arcadeDrive(mappedSpeed, mappedTurn);
	}

	/**
	 * Stops all of the motors of this {@link DriveTrain} and sets the target
	 * speeds to 0.
	 */
	public void stop() {
		drive.arcadeDrive(0, 0);
		driveSpeed = 0.0;
		turnSpeed = 0.0;
	}

	/**
	 * Sets the target drive speed of this {@link DriveTrain}. Actual speed of
	 * the motors does not change until update() is called.
	 * 
	 * @param speed
	 *            The new speed from -1.0 to 1.0
	 */
	public void setDriveSpeed(double speed) {
		speed = Transform.clamp(-1.0, 1.0, speed);
		driveSpeed = speed;
	}

	/**
	 * Sets the target turn speed of this {@link DriveTrain}, negative is left,
	 * positive is right. Actual speed of the motors does not change until
	 * update() is called.
	 * 
	 * @param speed
	 *            The new speed from -1.0 to 1.0
	 */
	public void setTurnSpeed(double speed) {
		// TODO This sign error is going to be carried everywhere
		// Invert speed to be more intuitive
		speed = -speed;

		speed = Transform.clamp(-1.0, 1.0, speed);
		turnSpeed = speed;
	}

	/**
	 * Put the <code>driveSpeed</code> and <code>turnSpeed</code> of this
	 * {@link DriveTrain} on the {@link SmartDashboard}.
	 */
	public void putToDashboard() {
		SmartDashboard.putNumber("Drive Speed", driveSpeed);
		SmartDashboard.putNumber("Turn Speed", turnSpeed);
	}
}
