/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.compressor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PnuematicSubsytem;

public class StopCompressorCommand extends CommandBase {
  /**
   * Creates a new CompressorCommand.
   */
  PnuematicSubsytem pnuematicSubsytem;

  public StopCompressorCommand(PnuematicSubsytem pnuematicSubsytem) {
    this.pnuematicSubsytem = pnuematicSubsytem;
    addRequirements(this.pnuematicSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pnuematicSubsytem.disable();
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
