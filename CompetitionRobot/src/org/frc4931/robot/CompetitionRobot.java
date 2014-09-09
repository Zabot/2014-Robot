package org.frc4931.robot;

import org.frc4931.robot.command.SetState;
import org.frc4931.robot.command.TwoState.State;
import org.frc4931.robot.command.drive.ModifiedDriveWithJoystick;
import org.frc4931.robot.command.drive.PIDDriveInterface;
import org.frc4931.robot.command.drive.PIDTurnInterface;
import org.frc4931.robot.command.groups.DriveAndScore;
import org.frc4931.robot.command.pneumatics.Pressurize;
import org.frc4931.robot.subsystems.Compressor;
import org.frc4931.robot.subsystems.DriveTrain;
import org.frc4931.robot.subsystems.IMU;
import org.frc4931.robot.subsystems.Nets;
import org.frc4931.robot.subsystems.Ranger;
import org.frc4931.robot.subsystems.RollerArm;
import org.frc4931.robot.subsystems.RollerArm.Arm;
import org.frc4931.robot.subsystems.RollerArm.Roller;
import org.frc4931.zach.drive.ContinuousMotor;
import org.frc4931.zach.drive.LimitedMotor;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CompetitionRobot extends IterativeRobot {
	public static final long START_TIME = System.currentTimeMillis();

	// TODO Phase these out
	public static boolean DRIVE_ENABLED = true;
	public static boolean COMPRESSOR_ENABLED = true;
	public static boolean ROLLER_ENABLED = true;
	public static boolean ARM_ENABLED = true;
	public static boolean NETS_ENABLED = true;

	private static final class PWMOutputs {
		public static final int DRIVE_MOTOR_FRONTLEFT = 1;
		public static final int DRIVE_MOTOR_FRONTRIGHT = 3;
		public static final int DRIVE_MOTOR_BACKLEFT = 2;
		public static final int DRIVE_MOTOR_BACKRIGHT = 4;

		public static final int ROLLER_MOTOR = 5;

		public static final int NET_MOTOR_LEFT = 6;
		public static final int NET_MOTOR_RIGHT = 7;
	}

	private static final class DigitalIO {
		public static final int COMPRESSOR_PRESSURESWITCH = 1;

		public static final int NET_SWITCH_RIGHT = 2;
		public static final int NET_SWITCH_LEFT = 3;

		// public static final int DIGITALIO4 = 4;
		// public static final int DIGITALIO5 = 5;
		// public static final int DIGITALIO6 = 6;

		public static final int NET_PROX_RIGHT = 7;
		public static final int NET_PROX_LEFT = 8;

		public static final int ULTRASONIC_RIGHT_PING = 9;
		public static final int ULTRASONIC_RIGHT_EHCO = 10;

		public static final int ULTRASONIC_LEFT_PING = 11;
		public static final int ULTRASONIC_LEFT_ECHO = 12;

		// public static final int DIGITALIO13 = 13;
		// public static final int DIGITALIO14 = 14;
	}

	private static final class SolenoidOutputs {
		public static final int ARM_LEFT_EXTEND = 1;
		public static final int ARM_LEFT_RETRACT = 2;
		public static final int ARM_RIGHT_EXTEND = 3;
		public static final int ARM_RIGHT_RETRACT = 4;
	}

	private static final class AnalogInput {
		public static final int GYRO_CHANNEL = 1;
		public static final int RANGER_CHANNEL = 2;
	}

	private static final class RelayOutputs {
		public static final int COMPRESSOR_RELAY = 1;
	}

	public void robotInit() {
		Subsystems.robot = this;

		// Instantiate the drive train
		Subsystems.driveTrain = new DriveTrain(
				PWMOutputs.DRIVE_MOTOR_FRONTLEFT,
				PWMOutputs.DRIVE_MOTOR_BACKLEFT,
				PWMOutputs.DRIVE_MOTOR_FRONTRIGHT,
				PWMOutputs.DRIVE_MOTOR_BACKRIGHT,
				ContinuousMotor.SpeedControllerType.TALON);

		// Instantiate the compressor
		Subsystems.compressor = new Compressor(RelayOutputs.COMPRESSOR_RELAY,
				DigitalIO.COMPRESSOR_PRESSURESWITCH);

		// Instantiate the nets
		Subsystems.nets = new Nets(
				new LimitedMotor(
						PWMOutputs.NET_MOTOR_LEFT,
						LimitedMotor.SpeedControllerType.VICTOR,
						DigitalIO.NET_SWITCH_LEFT,
						DigitalIO.NET_PROX_LEFT),
				new LimitedMotor(
						PWMOutputs.NET_MOTOR_RIGHT,
						LimitedMotor.SpeedControllerType.VICTOR,
						DigitalIO.NET_SWITCH_RIGHT,
						DigitalIO.NET_PROX_RIGHT));

		// Instantiate the roller arm
		Subsystems.rollerArm = new RollerArm(
				new Arm(SolenoidOutputs.ARM_LEFT_EXTEND,
						SolenoidOutputs.ARM_LEFT_RETRACT,
						SolenoidOutputs.ARM_RIGHT_EXTEND,
						SolenoidOutputs.ARM_RIGHT_RETRACT),
				new Roller(PWMOutputs.ROLLER_MOTOR,
						ContinuousMotor.SpeedControllerType.VICTOR));

		// Instantiate the front ultrasonic sensor
		Subsystems.ranger = new Ranger(AnalogInput.RANGER_CHANNEL);

		// Instantiate the IMU
		Subsystems.imu = new IMU(AnalogInput.GYRO_CHANNEL);

		// TODO Check if this is being used
		// Instantiate a driving pid loop
		Subsystems.pid = new PIDController(0.5, 0, 0, Subsystems.ranger,
				new PIDDriveInterface());

		// TODO Check if this is being used
		// Instantiate a turning pid loop
		Subsystems.turnPID = new PIDController(0.003, 0, 0.007, Subsystems.imu,
				new PIDTurnInterface());

		// Instantiate the left ultrasonic sensor
		Subsystems.leftUltrasonicSensor = new Ultrasonic(
				DigitalIO.ULTRASONIC_LEFT_PING,
				DigitalIO.ULTRASONIC_LEFT_ECHO,
				Ultrasonic.Unit.kInches);

		// Instantiate the right ultrasonic sensor
		Subsystems.rightUltrasonicSensor = new Ultrasonic(
				DigitalIO.ULTRASONIC_RIGHT_PING,
				DigitalIO.ULTRASONIC_RIGHT_EHCO,
				Ultrasonic.Unit.kInches);

		// Reset the IMU
		Subsystems.imu.init();

		// Set the left and right ultrasonic sensors to ping automatically
		Subsystems.leftUltrasonicSensor.setAutomaticMode(true);
		Subsystems.rightUltrasonicSensor.setAutomaticMode(true);

		// Attach commands to joystick buttons
		OperatorInterface.init();

		// Put initial values on the dashboard
		smartDashboardInit();
	}

	public void smartDashboardInit() {
		// Auto mode selector
		SmartDashboard.putNumber("Auto Mode", 0);

		// TODO Refine these values and make them constants for a joystick
		SmartDashboard.putNumber("DriveDeadZone", 0.08);
		SmartDashboard.putNumber("TurnDeadZone", 0.1);

		// TODO Refine these values and make them constants in DriveTrain
		SmartDashboard.putNumber("MinDriveSpeed", 0.3);
		SmartDashboard.putNumber("MaxDriveSpeed", 1.0);

		// TODO Refine these values and make them constants in DriveTrain
		SmartDashboard.putNumber("MinTurnSpeed", 0.3);
		SmartDashboard.putNumber("MaxTurnSpeed", 1.0);

		// TODO Refine these values and make them constants in DriveTrain
		SmartDashboard.putNumber("Range 1", 0.4);
		SmartDashboard.putNumber("Range 2", 0.8);
		SmartDashboard.putNumber("Range 3", 1.0);

		// TODO Refine these values and make them constants in DriveTrain
		SmartDashboard.putNumber("Max Delta 1", 1.0);
		SmartDashboard.putNumber("Max Delta 2", 0.1);
		SmartDashboard.putNumber("Max Delta 3", 0.01);

		SmartDashboard.putNumber("MaxApproachSpeed", 0.4);

		// TODO Determine if these are still being used
		SmartDashboard.putData("Drive PID", Subsystems.pid);
		SmartDashboard.putData("Turn PID", Subsystems.turnPID);

		/* Net Override Commands */
		SmartDashboard.putData("Close Left Net",
				new SetState(Subsystems.nets.leftNet, State.CLOSED));

		SmartDashboard.putData("Close Right Net",
				new SetState(Subsystems.nets.rightNet, State.CLOSED));

		SmartDashboard.putData("Open Left Net",
				new SetState(Subsystems.nets.leftNet, State.OPEN));

		SmartDashboard.putData("Open Right Net",
				new SetState(Subsystems.nets.rightNet, State.OPEN));

		// Manual arm overrides
		SmartDashboard.putData("Lower Roller Arm",
				new SetState(Subsystems.rollerArm.arm, State.DOWN));

		SmartDashboard.putData("Raise Roller Arm",
				new SetState(Subsystems.rollerArm.arm, State.UP));
	}

	public void robotPeriodic() {
		updateSmartDashboard();
		if (!isDisabled())
			enabledPeriodic();
	}

	public void updateSmartDashboard() {
		// Put ultrasonics on dashboard
		SmartDashboard.putNumber("Left Range",
				Subsystems.leftUltrasonicSensor.getRangeInches());

		SmartDashboard.putNumber("Right Range",
				Subsystems.rightUltrasonicSensor.getRangeInches());

		// Put subsystems on dashboard
		Subsystems.driveTrain.putToDashboard();
		Subsystems.compressor.putToDashboard();
		Subsystems.rollerArm.putToDashboard();
		Subsystems.ranger.putToDashboard();
		Subsystems.imu.putToDashboard();
	}

	public void enabledPeriodic() {
		// Update motors
		Subsystems.driveTrain.update();

		// Monitor pressure
		if (Subsystems.compressor.testPressure())
			Scheduler.getInstance().add(new Pressurize());

		// Run one iteration of the scheduler
		Scheduler.getInstance().run();
	}

	public void disabledPeriodic() {
		robotPeriodic();
	}

	public void teleopPeriodic() {
		robotPeriodic();

		// Schedule the drive command to run
		Scheduler.getInstance().add(new ModifiedDriveWithJoystick());
	}

	public void autonomousInit() {
		Scheduler.getInstance().add(new DriveAndScore());
	}

	public void autonomousPeriodic() {
		robotPeriodic();
	}

	/**
	 * Prints the specified string to the console along with the current robot
	 * time.
	 * 
	 * @param string
	 *            The string to print.
	 */
	public static void output(String string) {
		// TODO Add multiple levels of verbosity
		System.out.println(Math.floor(getTime() / 10.0d) / 100.0d + ":" + "\t"
				+ string);
	}

	/**
	 * Returns the time since the robot was powered on in milliseconds.
	 * 
	 * @return The time since the robot was powered on in milliseconds.
	 */
	public static long getTime() {
		return System.currentTimeMillis() - START_TIME;
	}
}
