/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CannonTiltSubsystem extends SubsystemBase {
  /**
   * Creates a new CannonTiltSubsystem.
   */
  public static WPI_TalonSRX cannonMotor = new WPI_TalonSRX(15);
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;
  public CannonTiltSubsystem() {

  }

  public void init(){
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    cannonMotor.configFactoryDefault();
    cannonMotor.setSelectedSensorPosition(0, 0, 0);
    cannonMotor.setNeutralMode(NeutralMode.Brake);
    cannonMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    cannonMotor.configOpenloopRamp(0.2);
    cannonMotor.configClosedloopRamp(0.2);
    cannonMotor.configNominalOutputForward(0, 0);
    cannonMotor.configNominalOutputReverse(0, 0);
    cannonMotor.configPeakOutputForward(1, 0);
    cannonMotor.configPeakOutputReverse(-1, 0);
    cannonMotor.setSensorPhase(true);
    cannonMotor.configReverseSoftLimitThreshold((int) (0 / Constants.kTick2Feet4Womf), 10);
    cannonMotor.configForwardSoftLimitThreshold((int) (175 / Constants.kTick2Feet4Womf), 10);
    
  }
  public void shootMode(){
    
    // Constants.setpointWomf = 6;
    // Constants.setpointShoot = 25;
    Constants.setpointShoot = -14;

    double sensorPosition = cannonMotor.getSelectedSensorPosition(0);
    double error = Constants.setpointShoot - sensorPosition;
    double dt = Timer.getFPGATimestamp() - Constants.lastTimestampShoot;
    if (Math.abs(error) < Constants.iLimitShoot) {
      Constants.errorSumShoot += error * dt;
    }
    double errorRate = (error - Constants.lastErrorShoot) / dt;
    Constants.kPShoot = (.1*1024)/error;
    Constants.kDShoot = Constants.kPShoot * 10;
    double outputSpeed = Constants.kPShoot * error + Constants.kIShoot * Constants.errorSumShoot + Constants.kDShoot * errorRate;
    cannonMotor.set(-outputSpeed);
    Constants.lastTimestampShoot = Timer.getFPGATimestamp();
    Constants.lastErrorShoot = error;
    System.out.println(sensorPosition);
  }
  public void climbMode(){
    
    // Constants.setpointWomf = 6;
    // Constants.setpointShoot = 25;
    Constants.setpointShoot = 0.1 * Constants.kCannonTick2Deg;

    double sensorPosition = cannonMotor.getSelectedSensorPosition(0) * Constants.kCannonTick2Deg;
    double error = Constants.setpointShoot - sensorPosition;
    double dt = Timer.getFPGATimestamp() - Constants.lastTimestampShoot;
    if (Math.abs(error) < Constants.iLimitShoot) {
      Constants.errorSumShoot += error * dt;
    }
    double errorRate = (error - Constants.lastErrorShoot) / dt;
    Constants.kPShoot = (.1*1024)/error;
    Constants.kDShoot = Constants.kPShoot * 10;
    double outputSpeed = Constants.kPShoot * error + Constants.kIShoot * Constants.errorSumShoot + Constants.kDShoot * errorRate;
    cannonMotor.set(-outputSpeed);
    Constants.lastTimestampShoot = Timer.getFPGATimestamp();
    Constants.lastErrorShoot = error;
    System.out.println(sensorPosition);
  }

  public void womfMode(){

      Constants.setpointShoot = -15 * Constants.kCannonTick2Deg;

      double sensorPosition = cannonMotor.getSelectedSensorPosition(0) * Constants.kCannonTick2Deg;
      double error = Constants.setpointShoot - sensorPosition;
      double dt = Timer.getFPGATimestamp() - Constants.lastTimestampShoot;
      if (Math.abs(error) < Constants.iLimitShoot) {
        Constants.errorSumShoot += error * dt;
      }
      double errorRate = (error - Constants.lastErrorShoot) / dt;
      Constants.kPShoot = (.1*1024)/error;
      Constants.kDShoot = Constants.kPShoot * 10;
      double outputSpeed = Constants.kPShoot * error + Constants.kIShoot * Constants.errorSumShoot + Constants.kDShoot * errorRate;
      cannonMotor.set(-outputSpeed);
      Constants.lastTimestampShoot = Timer.getFPGATimestamp();
      Constants.lastErrorShoot = error;
      System.out.println(sensorPosition);
    }
   
  
  public void setIntake(){
    Constants.setpointShoot = -50 * Constants.kCannonTick2Deg;

    double sensorPosition = cannonMotor.getSelectedSensorPosition(0) * Constants.kCannonTick2Deg;
    double error = Constants.setpointShoot - sensorPosition;
    double dt = Timer.getFPGATimestamp() - Constants.lastTimestampShoot;
    if (Math.abs(error) < Constants.iLimitShoot) {
      Constants.errorSumShoot += error * dt;
    }
    double errorRate = (error - Constants.lastErrorShoot) / dt;
    Constants.kPShoot = (.1*1024)/error;
    Constants.kDShoot = Constants.kPShoot * 10;
    double outputSpeed = Constants.kPShoot * error + Constants.kIShoot * Constants.errorSumShoot + Constants.kDShoot * errorRate;
    cannonMotor.set(-outputSpeed);
    Constants.lastTimestampShoot = Timer.getFPGATimestamp();
    Constants.lastErrorShoot = error;
    System.out.println(sensorPosition);
  }
  public void stop(){
    cannonMotor.set(0.0);
  }
  public void test(){
    cannonMotor.set(-.35);
  }
  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}