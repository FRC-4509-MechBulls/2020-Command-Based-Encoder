package frc.robot.commands.driving;

import java.util.function.DoubleSupplier;

import frc.robot.Robot;
import frc.robot.subsystems.DrivingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DirectDriveCommand extends CommandBase {

	// private DrivingSubsystem dSub = new DrivingSubsystem();
	private final DrivingSubsystem dSub;

	private final DoubleSupplier forward;
	private final DoubleSupplier rotation;


	public DirectDriveCommand(DrivingSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
		// Use addRequirements() here to declare subsystem dependencies.
		dSub = subsystem;
		this.forward = forward;
		this.rotation = rotation;
		addRequirements(dSub);
	}

	public void initialize() {
		// Make sure the controller is initialized before we try to use it.

	}

	public void execute() {
		dSub.drive(forward.getAsDouble(), rotation.getAsDouble()); // Use input directly from the controller
	}

	public boolean isFinished() {
		return false; // We don't want the command to stop, we want it to be interrupted.
	}

	protected void end() {
		Robot.drivingSubsystem.stop();
	}

}