package org.frc4931.robot.subsystems;

import org.frc4931.zach.io.Accel;

import edu.wpi.first.wpilibj.ADXL345_I2C.Axes;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IMU extends SubsystemBase implements PIDSource {
	private final Gyro gyro;
	private final Accel accel;

	public IMU(
			int gyroChannel) {
		gyro = new Gyro(gyroChannel);
		accel = new Accel(Accel.FOURG);
	}

	public double getContinuousAngle() {
		return gyro.getAngle();
	}

	public double getBoundedAngle() {
		return gyro.getAngle() % 360;
	}

	public double getAcceleration(Axes axis) {
		return accel.getAcceleration(axis);
	}

	public Gyro getGyro() {
		return gyro;
	}

	public Accel getAccel() {
		return accel;
	}

	public void resetGyro() {
		gyro.reset();
	}

	public void resetAccel() {
		accel.zero(Accel.X);
		accel.zero(Accel.Y);
	}

	public void reset() {
		resetGyro();
		resetAccel();
	}

	public void init() {
		reset();
	}

	public void putToDashboard() {
		SmartDashboard.putData("Gyroscope", gyro);
		SmartDashboard.putNumber("Angle", getContinuousAngle());
		SmartDashboard.putNumber("Accelerometer X",
				accel.getAcceleration(Accel.X));
		SmartDashboard.putNumber("Accelerometer Y",
				accel.getAcceleration(Accel.Y));
		SmartDashboard.putNumber("Accelerometer Z",
				accel.getAcceleration(Accel.Z));
	}

	public double pidGet() {
		if (getContinuousAngle() < 0) {
			return (getContinuousAngle() % 360) + 360;
		} else {
			return getContinuousAngle() % 360;
		}
	}

}
