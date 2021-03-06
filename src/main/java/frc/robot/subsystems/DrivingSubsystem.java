/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivingSubsystem extends SubsystemBase {

	public static DifferentialDrive drive;
	public static WPI_TalonSRX leftFrontDriveTalon;
	public static WPI_TalonSRX leftBackDriveTalon;
	public static WPI_TalonSRX rightFrontDriveTalon;
	public static WPI_TalonSRX rightBackDriveTalon;

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void drive(double leftSpeed, double rightSpeed) {

		if (Math.abs(leftSpeed) > 1.0)
		leftSpeed = Math.abs(leftSpeed) / leftSpeed; // if the value given was too high, set it to the max
		leftSpeed *= Constants.baseDriveSpeed; // scale down the speed

		if (Math.abs(rightSpeed) > 1)
			rightSpeed = Math.abs(rightSpeed) / rightSpeed; // if the value given was too high, set it to the max
			rightSpeed *= Constants.baseDriveSpeed; // scale down the speed

		drive.tankDrive(leftSpeed, rightSpeed); // function provided by the drivetrain. controls y and turn speed at the
												// // same time.
	}

	public static void initDrive() {
		leftFrontDriveTalon = new WPI_TalonSRX(Constants.LEFT_FRONT_DRIVE_TALON_PORT);
		leftBackDriveTalon = new WPI_TalonSRX(Constants.LEFT_BACK_DRIVE_TALON_PORT);
		rightFrontDriveTalon = new WPI_TalonSRX(Constants.RIGHT_FRONT_DRIVE_TALON_PORT);
		rightBackDriveTalon = new WPI_TalonSRX(Constants.RIGHT_BACK_DRIVE_TALON_PORT);
		leftBackDriveTalon.setNeutralMode(NeutralMode.Brake);
		leftFrontDriveTalon.setNeutralMode(NeutralMode.Brake);
		rightBackDriveTalon.setNeutralMode(NeutralMode.Brake);
		rightFrontDriveTalon.setNeutralMode(NeutralMode.Brake);
		leftBackDriveTalon.follow(leftFrontDriveTalon);
		rightBackDriveTalon.follow(rightFrontDriveTalon);
		leftFrontDriveTalon.configOpenloopRamp(1.0);
		leftFrontDriveTalon.configClosedloopRamp(0.0);
		rightFrontDriveTalon.configOpenloopRamp(1.0);
		rightFrontDriveTalon.configClosedloopRamp(0.0);
		drive = new DifferentialDrive(leftFrontDriveTalon, rightFrontDriveTalon);

		drive.setDeadband(0.02);
	}
	public void test(){
		// leftFrontDriveTalon.set(-0.5);
		// // leftBackDriveTalon.set(-0.5);
		// rightFrontDriveTalon.set(0.5);
		// rightBackDriveTalon.set(0.5);
		// drive.arcadeDrive(0.5, 0.0);

	}
	// public void autonomousDrive(){
	// 	Constants.setpointWomf = 50;
	// 	double sensorPosition = leftBackDriveTalon.getSelectedSensorPosition(0) * Constants.kTick2Feet4Womf;
	// 	double error = Constants.setpointWomf - sensorPosition;
	// 	double dt = Timer.getFPGATimestamp() - Constants.lastTimestampWomf;
	// 	if (Math.abs(error) < Constants.iLimitWomf) {
	// 	  Constants.errorSumWomf += error * dt;
	// 	}
	// 	double errorRate = (error - Constants.lastErrorWomf) / dt;
	// 	double outputSpeed = Constants.kPWomf * error + Constants.kIWomf * Constants.errorSumWomf + Constants.kDWomf * errorRate;
	// 	leftBackDriveTalon.set(outputSpeed);
	// 	leftFrontDriveTalon.set(outputSpeed);
	// 	rightBackDriveTalon.set(-outputSpeed);
	// 	rightFrontDriveTalon.set(-outputSpeed);
	
	// 	Constants.lastTimestampWomf = Timer.getFPGATimestamp();
	// 	Constants.lastErrorWomf = error;
	// }
	// Directly set the speed of the talons to 0. If a command that sets the speed
	// is still running, this won't stop it.
	public void stop() {
		leftFrontDriveTalon.set(0);
		rightFrontDriveTalon.set(0);
	}

}
