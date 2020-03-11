package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.climber.TurnOffClimberCommand;
import frc.robot.commands.compressor.StartCompressorCommand;
import frc.robot.commands.compressor.StopCompressorCommand;
import frc.robot.commands.driving.DirectDriveCommand;
import frc.robot.commands.index.IndexIntakeCommand;
import frc.robot.commands.index.IndexShooterCommand;
import frc.robot.commands.index.StopIndexCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeOffCommand;
import frc.robot.commands.shooter.ShooterOnCommand;
import frc.robot.commands.tilt.CannonClimbMode;
import frc.robot.commands.tilt.CannonIntakeMode;
import frc.robot.commands.tilt.CannonShootMode;
import frc.robot.commands.tilt.StopTiltCommand;
import frc.robot.subsystems.CannonTiltSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivingSubsystem;
import frc.robot.subsystems.IntakeAndShootSubsystem;
import frc.robot.subsystems.PnuematicSubsystem;
import frc.robot.subsystems.WomfSubsystem;

public class RobotContainer {

    // public XboxController1 controller1;
    Joystick joshJoystick = new Joystick(2);
    // XboxController controller1 = new
    // XboxController(Constants.XBOX_CONTROLLER_1_PORT);
    Joystick zachRightJoystick = new Joystick(0);
    Joystick zachLeftJoystick = new Joystick(1);

    DrivingSubsystem drivingSubsystem = new DrivingSubsystem();
    ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    PnuematicSubsystem pnuematicSubsystem = new PnuematicSubsystem();
    IntakeAndShootSubsystem intakeAndShootSubsystem = new IntakeAndShootSubsystem();
    WomfSubsystem womfSubsystem = new WomfSubsystem();
    CannonTiltSubsystem cannonTiltSubsystem = new CannonTiltSubsystem();

    public RobotContainer() {
        configureButtonBindings();

        drivingSubsystem.setDefaultCommand(
                new DirectDriveCommand(drivingSubsystem, () -> getLeftDrive(), () -> getRightDrive()));

    }

    public double getRightDrive() {
        return zachRightJoystick.getY();
    }

    public double getLeftDrive() {
        return zachLeftJoystick.getY();
    }

    private void configureButtonBindings() {
        //womfButton, climberButton, cannonShoot, cannonIntake, climbModeCannon, cannonTiltIntake, cannonTiltShoot
        final JoystickButton cannonTiltIntake = new JoystickButton(joshJoystick, 8);
        final JoystickButton cannonTiltShoot = new JoystickButton(joshJoystick, 11);
        final JoystickButton climberButton = new JoystickButton(joshJoystick, 6);
        final JoystickButton climbModeCannon = new JoystickButton(joshJoystick, 12);
        final JoystickButton cannonShoot = new JoystickButton(joshJoystick, 2);
        final JoystickButton cannonIntake= new JoystickButton(joshJoystick, 7);
        final JoystickButton indexerShoot= new JoystickButton(joshJoystick, 1);
        final JoystickButton indexerIntake = new JoystickButton(joshJoystick, 9);
        final JoystickButton compressor = new JoystickButton(zachRightJoystick, 1);
        climberButton.whenPressed(new ClimberCommand(climberSubsystem));
        climberButton.whenReleased(new TurnOffClimberCommand(climberSubsystem));
        cannonTiltIntake.whenPressed(new CannonIntakeMode(cannonTiltSubsystem));
        cannonTiltIntake.whenReleased(new StopTiltCommand(cannonTiltSubsystem));
        climbModeCannon.whenPressed(new CannonClimbMode(cannonTiltSubsystem));
        climbModeCannon.whenReleased(new StopTiltCommand(cannonTiltSubsystem));
        cannonTiltShoot.whenPressed(new CannonShootMode(cannonTiltSubsystem));
        indexerIntake.whenPressed(new IndexIntakeCommand(intakeAndShootSubsystem));
        indexerIntake.whenReleased(new StopIndexCommand(intakeAndShootSubsystem));
        compressor.whenPressed(new StartCompressorCommand(pnuematicSubsystem));
        compressor.whenReleased(new StopCompressorCommand(pnuematicSubsystem));
        // cannonTiltShoot.whenReleased(new StopTiltCommand(cannonTiltSubsystem));
        cannonShoot.whenPressed(new ShooterOnCommand(intakeAndShootSubsystem));
        cannonShoot.whenReleased(new IntakeOffCommand(intakeAndShootSubsystem));
        cannonIntake.whenPressed(new IntakeCommand(intakeAndShootSubsystem));
        cannonIntake.whenReleased(new StopTiltCommand(cannonTiltSubsystem).alongWith(new IntakeOffCommand(intakeAndShootSubsystem)));
        indexerShoot.whenPressed(new IndexShooterCommand(intakeAndShootSubsystem));
        indexerShoot.whenReleased(new StopIndexCommand(intakeAndShootSubsystem));
    }
  
  
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    //   // An ExampleCommand will run in autonomous
    //   return autoDefault;
    // }
 }