/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeAndShootSubsystem;

public class IntakeCommand extends CommandBase {

  /**
   * Creates a new IntakeCommand.
   */
  IntakeAndShootSubsystem intakeAndShootSubsystem;

  public IntakeCommand(IntakeAndShootSubsystem subsystem) {
    intakeAndShootSubsystem = subsystem;

    addRequirements(intakeAndShootSubsystem);




}


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeAndShootSubsystem.enable(1.0,0.8); 
 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
