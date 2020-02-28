/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * The Constants is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class Constants {



	public static final int LEFT_FRONT_DRIVE_TALON_PORT  = 3;
	public static final int LEFT_BACK_DRIVE_TALON_PORT   = 6;
	public static final int RIGHT_FRONT_DRIVE_TALON_PORT = 11;
	public static final int RIGHT_BACK_DRIVE_TALON_PORT  = 2;
	public static final double baseDriveSpeed = 0.90;

	public static final int XBOX_CONTROLLER_1_PORT = 0;
    public static final int XBOX_CONTROLLER_2_PORT = 1;
	public static final int CLIMBER_SOLENOID_PORT = 0;
	public static final int INTAKE_SOLENOID_PORT =3;
	public static Solenoid climberSolenoid = new Solenoid(CLIMBER_SOLENOID_PORT);
	public static Solenoid intakeSolenoid = new Solenoid(INTAKE_SOLENOID_PORT);
	public static final double kCannonTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;
	public static final double kPWomf = 0.04;
    public static final double kIWomf = 0.04;
    public static final double kDWomf = 0.5;
    public static final double iLimitWomf = 1;
    public static final double kTick2Feet4Womf = 1.0 / 128 * 6 * Math.PI / 12;
	public static double setpointWomf = 0;
	
    public static double errorSumWomf = 0;
    public static double lastTimestampWomf = 0;
    public static double lastErrorWomf = 0;
	public static double setpointShoot = 0;
    public static double errorSumShoot = 0;
    public static double lastTimestampShoot = 0;
    public static double lastErrorShoot = 0;
	public static final double kPShoot = 0.04;
    public static final double kIShoot = 0.04;
	public static final double kDShoot = 0.04;
	public static final double iLimitShoot = 1;

	


	

	


}
