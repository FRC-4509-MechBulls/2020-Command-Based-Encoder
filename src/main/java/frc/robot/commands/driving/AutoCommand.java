/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driving;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonTiltSubsystem;
import frc.robot.subsystems.DrivingSubsystem;
import frc.robot.subsystems.IntakeAndShootSubsystem;

public class AutoCommand extends CommandBase {
  /**
   * Creates a new AutoCommand.
   */
  DrivingSubsystem drivingSubsystem;
  CannonTiltSubsystem cannonTiltSubsystem;
  IntakeAndShootSubsystem intakeAndShootSubsystem;
  Timer timer;

  public AutoCommand(DrivingSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivingSubsystem = subsystem;
    addRequirements(drivingSubsystem);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cannonTiltSubsystem.shootMode();
    
    while (timer.get() < 5) {
      intakeAndShootSubsystem.enable(0.0, -.95);
      intakeAndShootSubsystem.index(-1.0);
    }
    DrivingSubsystem.drive.arcadeDrive(-.75, 0);// drive backwards at 3/4 speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 10; // end the command if we have run for at least 3 seconds
  }
}