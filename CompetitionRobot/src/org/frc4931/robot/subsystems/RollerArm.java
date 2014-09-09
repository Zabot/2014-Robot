package org.frc4931.robot.subsystems;

import org.frc4931.robot.command.Requireable;
import org.frc4931.robot.command.TwoState;
import org.frc4931.zach.drive.ContinuousMotor;
import org.frc4931.zach.drive.Solenoid;

import edu.wpi.first.wpilibj.command.Subsystem;

public class RollerArm extends SubsystemBase {

	public final Arm arm;
	public final Roller roller;

	public RollerArm(
			int leftSolenoidExtend,
			int leftSolenoidRetract,
			int rightSolenoidExtend,
			int rightSolenoidRetract,
			int rollerMotorChannel,
			ContinuousMotor.SpeedControllerType rollerMotorType) {

		this(new Arm(leftSolenoidExtend,
				leftSolenoidRetract,
				rightSolenoidExtend,
				rightSolenoidRetract),
				new Roller(rollerMotorChannel, rollerMotorType));
	}

	public RollerArm(
			Arm arm,
			Roller roller) {
		this.arm = arm;
		this.roller = roller;
		arm.raise();
	}

	public void putToDashboard() {
	}

	public static final class Roller implements Requireable {
		public static final double SPEED = 0.5d;

		private final ContinuousMotor rollerMotor;

		public Roller(
				int rollerMotor,
				ContinuousMotor.SpeedControllerType type) {
			this.rollerMotor = new ContinuousMotor(rollerMotor, type);
		}

		public void rollIn() {
			rollerMotor.setSpeed(SPEED);
		}

		public void rollOut() {
			rollerMotor.setSpeed(-SPEED);
		}

		public void stop() {
			rollerMotor.stop();
		}

		public Subsystem getSubsystem() {
			return rollerMotor.getSubsystem();
		}
	}

	public static final class Arm implements TwoState, Requireable {
		private final Subsystem subsystem = new SubsystemBase() {
		};
		private final Solenoid leftSolenoid;
		private final Solenoid rightSolenoid;

		private State logicalState = State.UNKNOWN;

		public Arm(
				int leftSolenoidExtend,
				int leftSolenoidRetract,
				int rightSolenoidExtend,
				int rightSolenoidRetract) {
			leftSolenoid = new Solenoid(leftSolenoidExtend, leftSolenoidRetract);
			rightSolenoid = new Solenoid(rightSolenoidExtend,
					rightSolenoidRetract);
		}

		public void raise() {
			leftSolenoid.retract();
			rightSolenoid.retract();
			logicalState = State.ONE;
		}

		public void lower() {
			leftSolenoid.extend();
			rightSolenoid.extend();
			logicalState = State.TWO;
		}

		public boolean isDown() {
			return leftSolenoid.isExtended() && rightSolenoid.isExtended();
		}

		public boolean isUp() {
			return leftSolenoid.isRetracted() && rightSolenoid.isRetracted();
		}

		public void setStateOne(double speed) {
			raise();
		}

		public void setStateTwo(double speed) {
			lower();
		}

		public State getPhysicalState() {
			if (isUp()) {
				return State.ONE;
			} else if (isDown()) {
				return State.TWO;
			} else {
				return State.UNKNOWN;
			}
		}

		public State getLogicalState() {
			return logicalState;
		}

		public boolean isContinous() {
			return false;
		}

		public void stop() {
		}

		public String getName() {
			return "RollerArm.Arm";
		}

		public Subsystem getSubsystem() {
			return subsystem;
		}
	}
}
