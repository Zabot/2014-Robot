package org.frc4931.zach.utils;

public class Transform {
	/**
	 * Maps one value in a range of values to another range of values.
	 * 
	 * @param inMin
	 *            The minimum of the input range.
	 * @param inMax
	 *            The maximum of the input range.
	 * @param outMin
	 *            The minimum of the output range.
	 * @param outMax
	 *            The maximum of the output range.
	 * @param value
	 *            The value to map.
	 * @return The mapped value.
	 */
	public static double map(double inMin, double inMax, double outMin,
			double outMax, double value) {
		return (value - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
	}

	/**
	 * Clamps a value between two other values
	 * 
	 * @param min
	 *            The minimum allowed value.
	 * @param max
	 *            The maximum allowed value.
	 * @param value
	 *            The value to clamp
	 * @return The clamped value.
	 */
	public static double clamp(double min, double max, double value) {
		value = Math.min(max, value);
		value = Math.max(min, value);
		return value;
	}

	/**
	 * Clamps a value between two other values
	 * 
	 * @param min
	 *            The minimum allowed value.
	 * @param max
	 *            The maximum allowed value.
	 * @param value
	 *            The value to clamp
	 * @return The clamped value.
	 */
	public static int clamp(int min, int max, int value) {
		value = Math.min(max, value);
		value = Math.max(min, value);
		return value;
	}

	/**
	 * Returns the sign of a number
	 * 
	 * @param n
	 *            The number to check
	 * @return 1 if n is positive, -1 if n is negative; 0 otherwise
	 */
	public static int signum(double n) {
		if (n < 0)
			return -1;
		else if (n > 0)
			return 1;
		else
			return 0;
	}
}
