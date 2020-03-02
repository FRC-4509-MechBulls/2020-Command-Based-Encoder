/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driving;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CannonTiltSubsystem;
import frc.robot.subsystems.DrivingSubsystem;
import frc.robot.subsystems.IntakeAndShootSubsystem;
import frc.robot.commands.shooter.ShooterOnCommand;
import frc.robot.commands.tilt.CannonShootMode;

public class AutoCommand extends CommandBase {
  /**
   * Creates a new AutoCommand.
   */
  DrivingSubsystem drivingSubsystem;
  CannonTiltSubsystem cannonTiltSubsystem;
  IntakeAndShootSubsystem intakeAndShootSubsystem;
  Timer timer;

  public AutoCommand(DrivingSubsystem subsystem, CannonTiltSubsystem cannonTiltSubsystem,
      IntakeAndShootSubsystem intakeAndShootSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivingSubsystem = subsystem;
    this.cannonTiltSubsystem = cannonTiltSubsystem;
    this.intakeAndShootSubsystem = intakeAndShootSubsystem;
    addRequirements(drivingSubsystem);
    addRequirements(cannonTiltSubsystem);
    addRequirements(intakeAndShootSubsystem);
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
    // SequentialCommandGroup shootingGroup = new SequentialCommandGroup(
    //     new ShooterOnCommand(intakeAndShootSubsystem));
  
    // intakeAndShootSubsystem.enable(0.0, -1);

    // SequentialCommandGroup shootingGroup = new SequentialCommandGroup(
    // new CannonShootMode(cannonTiltSubsystem).andThen(new
    // ShooterOnCommand(intakeAndShootSubsystem)));

    // // cannonTiltSubsystem.shootMode().andThen(new
    // // ShooterOnCommand(intakeAndShootSubsystem));
    // // while (timer.get() < 8) {
    // shootingGroup.execute();
    // // }

    drivingSubsystem.drive.arcadeDrive(.5, 0);// drive straight at half
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 3; // end the command if we have run for at least 3 seconds
  }
}