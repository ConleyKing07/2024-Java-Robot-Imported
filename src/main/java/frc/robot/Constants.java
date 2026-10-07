// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Motor Controller Constants
	public static final int FRONT_RIGHT_MOTOR_CONTROLLER_ID = 1;
	public static final int BACK_RIGHT_MOTOR_CONTROLLER_ID = 2;
	public static final int BACK_LEFT_MOTOR_CONTROLLER_ID = 3;
	public static final int FRONT_LEFT_MOTOR_CONTROLLER_ID = 4;
	public static final int RIGHT_CHAIN_MOTOR_CONTROLLER_ID = 5;
	public static final int LEFT_CHAIN_MOTOR_CONTROLLER_ID = 6;
	public static final int RIGHT_LAUNCHER_MOTOR_CONTROLLER_ID = 7;
	public static final int LEFT_LAUNCHER_MOTOR_CONTROLLER_ID = 8;
	public static final int TOP_LAUNCHER_MOTOR_CONTROLLER_ID = 9;
	public static final int RIGHT_ARM_MOTOR_CONTROLLER_ID = 10;
	public static final int LEFT_ARM_MOTOR_CONTROLLER_ID = 11;

    //Motor speeds
	public static final double AUTO_DRIVE_SPEED = 1.0;
	public static final double DRIVE_SPEED = 1.0;
	public static final double DRIVE_SPEED_FAST = 1.0;
	public static final double DRIVE_SPEED_SLOW = 0.5;
	public static final double ARM_SPEED = 1.0;
	public static final double SHOOTER_ARM_SPEED = 1.0;
	public static final double INTAKE_SPEED = 0.98;
	public static final double FAST_SHOOT_SPEED = 1.0;
	public static final double NORMAL_SHOOT_SPEED = 0.65;
	public static final double SLOW_SHOOT_SPEED = 0.2;

	//Tank Drive Values
	public static final double DRIVE_MULTIPLIER = 1.0;
	public static final double TURN_MULTIPLIER = 0.75;
	public static final double TURN_IN_PLACE_MULTIPLIER = 0.9;
	public static final double MAX_MOTOR_OUTPUT = 0.95;

	//Drive control port IDs
	public static final int DRIVER_XBOX_PORT = 0;
	public static final int SHOOTER_XBOX_PORT = 1;
	public static final int LEFT_JOYSTICK = 1;
	public static final int RIGHT_JOYSTICK = 2;
}
