/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.driving.AutoCommand;
import frc.robot.subsystems.CannonTiltSubsystem;
import frc.robot.subsystems.DrivingSubsystem;
import frc.robot.subsystems.WomfSubsystem;
import edu.wpi.first.wpilibj.Timer;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  public static final DrivingSubsystem drivingSubsystem = new DrivingSubsystem();
  private static final String kCustomAuto = "My Auto";
  private final Timer m_timer = new Timer();
  Command auto;
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  static RobotContainer oi = new RobotContainer();
  CannonTiltSubsystem cannonTiltSubsystem = new CannonTiltSubsystem();
  WomfSubsystem womfSubsystem = new WomfSubsystem();
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
   * The device will be automatically initialized with default parameters.
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.531, 0.343, 0.14);
  private final Color kYellowTarget = ColorMatch.makeColor(0.31597, 0.57, 0.11425);
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    auto = new AutoCommand(drivingSubsystem);
    auto.initialize();
    Robot.oi = new RobotContainer();
    womfSubsystem.colorMatcher.setConfidenceThreshold(1.0);
    womfSubsystem.colorMatcher.addColorMatch(kBlueTarget);
    womfSubsystem.colorMatcher.addColorMatch(kGreenTarget);
    womfSubsystem.colorMatcher.addColorMatch(kRedTarget);
    womfSubsystem.colorMatcher.addColorMatch(kYellowTarget);
    drivingSubsystem.initDrive();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", cannonTiltSubsystem.cannonMotor.getSelectedSensorPosition(0) * Constants.kCannonTick2Deg);
  }

  @Override
  public void disabledInit() {
    // Robot.drivingSubsystem.stop();

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().run();

    // auto.schedule();
    if (auto != null) auto.schedule();
    if(auto.isFinished() == true){
      auto.cancel();
    }
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    if(auto.isFinished() == true){
      auto.cancel();
    }
    
        switch (m_autoSelected) {
    case kCustomAuto:
    // Put custom auto code here
    break;
    case kDefaultAuto:
    default:
    // Put default auto code here
    break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopInit() {
    cannonTiltSubsystem.init();

    // TODO Auto-generated method stub
    super.teleopInit();
    auto.cancel();
  }
  @Override
  public void teleopPeriodic() {
    
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B':
        womfSubsystem.targetColor = kBlueTarget;
        SmartDashboard.putString("Blue", "B");
        break;
      case 'G':
        womfSubsystem.targetColor = kGreenTarget;

        // Green case code
        SmartDashboard.putString("Green", "G");

        break;
      case 'R':
        womfSubsystem.targetColor = kRedTarget;

        SmartDashboard.putString("Red", "R");

        break;
      case 'Y':
        womfSubsystem.targetColor = kYellowTarget;

        SmartDashboard.putString("Yellow", "Y");

        break;
      default:
        // This is corrupt data
        break;
      }
    } else {
      // Code for no data received yet
    }
    CommandScheduler.getInstance().run();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}