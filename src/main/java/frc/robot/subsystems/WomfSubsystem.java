/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WomfSubsystem extends SubsystemBase {
  /**
   * Creates a new WomfSubsystem.
   */
  public WomfSubsystem() {

  }
  
  public static Servo womfServo = new Servo(0); //PWM Port on roboRio
  IntakeAndShootSubsystem intakeAndShootSubsystem = new IntakeAndShootSubsystem();
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  public final ColorMatch colorMatcher = new ColorMatch();
  Color detectedColor = colorSensor.getColor();
  double IR = colorSensor.getIR();

  /**
   * Note: Any example colors should be calibrated as the user needs, these are
   * here as a basic example.
   */
  public Color targetColor = ColorMatch.makeColor(0.0, 0.0, 0.0);

  boolean stop = false;

  public void setServo(){
      womfServo.set(1.0);
  }
  public void setServoBack(){
    womfServo.set(-1.0);
  }
  public void stage1(){
    Constants.setpointWomf = 50;
    double sensorPosition = intakeAndShootSubsystem.flywheel1.getSelectedSensorPosition(0) * Constants.kTick2Feet4Womf;
    double error = Constants.setpointWomf - sensorPosition;
    double dt = Timer.getFPGATimestamp() - Constants.lastTimestampWomf;
    if (Math.abs(error) < Constants.iLimitWomf) {
      Constants.errorSumWomf += error * dt;
    }
    double errorRate = (error - Constants.lastErrorWomf) / dt;
    double outputSpeed = Constants.kPWomf * error + Constants.kIWomf * Constants.errorSumWomf + Constants.kDWomf * errorRate;
    intakeAndShootSubsystem.flywheel1.set(outputSpeed);
    Constants.lastTimestampWomf = Timer.getFPGATimestamp();
    Constants.lastErrorWomf = error;
  }
  public void stage2() {

    Color detectedColor = colorSensor.getColor();
    int proximity = colorSensor.getProximity();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    colorMatcher.matchClosestColor(detectedColor);
    // for targetColor, in gamedata in setup type B, Y, R, G for the colors it needs
    // to detect
    if (match.color == targetColor) {
      intakeAndShootSubsystem.flywheel1.set(0);
      stop = true;

    } else if (match.color != targetColor) {
      intakeAndShootSubsystem.flywheel1.set(0.2);
      if (stop) {
        intakeAndShootSubsystem.flywheel1.set(0.0);
      }

    }

  }

  public void disable() {
    intakeAndShootSubsystem.flywheel1.set(0);
    stop = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
