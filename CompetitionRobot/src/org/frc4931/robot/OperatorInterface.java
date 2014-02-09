package org.frc4931.robot;

import org.frc4931.robot.command.drive.ChangeDriveMode;
import org.frc4931.robot.command.net.CloseNets;
import org.frc4931.robot.command.net.OpenNets;
import org.frc4931.robot.command.pneumatics.LowerArm;
import org.frc4931.robot.command.pneumatics.RaiseArm;
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
	public static void init(){
		//Initialize Joysticks
		joysticks = new FlightStick[2];
		joysticks[0] = new LogitechPro(1);
		joysticks[1] = new LogitechAttack(2);
		
		//Initialize Commands
		initButtonCommands();
	}
	/**
	 * Initializes commands on joystick buttons.
	 */
	private static void initButtonCommands(){
		joysticks[0].buttons[7].whenPressed(new ChangeDriveMode(0));
		joysticks[0].buttons[8].whenPressed(new ChangeDriveMode(1));
		joysticks[0].buttons[9].whenPressed(new ChangeDriveMode(2));
		joysticks[0].buttons[11].whenPressed(new ChangeDriveMode(3));
		joysticks[0].buttons[5].whenPressed(new CloseNets(0.5d));
		joysticks[0].buttons[6].whenPressed(new OpenNets(0.5d));
		joysticks[0].buttons[1].whenPressed(new LowerArm());
		joysticks[0].buttons[2].whenPressed(new RaiseArm());
		joysticks[0].buttons[3].whenPressed(new RollIn());
		joysticks[0].buttons[4].whenPressed(new RollOut());
		joysticks[0].buttons[12].whenPressed(new StopRoller());
	}
}
