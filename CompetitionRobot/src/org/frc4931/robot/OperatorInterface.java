package org.frc4931.robot;

import org.frc4931.robot.command.Toggle;
import org.frc4931.robot.command.roller.RollIn;
import org.frc4931.robot.command.roller.RollOut;
import org.frc4931.robot.command.roller.StopRoller;
import org.frc4931.zach.control.FlightStick;
import org.frc4931.zach.control.LogitechAttack;
import org.frc4931.zach.control.LogitechPro;

public class OperatorInterface {
	public static FlightStick[] joysticks;

	/**
	 * Sets up the operator interface
	 */
	public static void init() {
		// Instantiate joysticks
		joysticks = new FlightStick[2];
		joysticks[0] = new LogitechPro(1);
		joysticks[1] = new LogitechAttack(2);

		// Initialize Commands
		initButtonCommands();
	}

	// TODO Create multiple user profiles using constant classes
	private static void initButtonCommands() {
		joysticks[0].buttons[3].whenPressed(new RollIn());
		joysticks[0].buttons[3].whenReleased(new StopRoller());

		joysticks[0].buttons[5].whenPressed(new RollOut());
		joysticks[0].buttons[5].whenReleased(new StopRoller());

		joysticks[0].buttons[1]
				.whenPressed(new Toggle(Subsystems.rollerArm.arm));

		joysticks[0].buttons[6].whenPressed(new Toggle(Subsystems.nets));
	}
}
