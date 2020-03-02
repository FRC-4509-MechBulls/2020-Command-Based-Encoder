package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.StartCompressorCommand;
import frc.robot.commands.StopCompressorCommand;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.climber.TurnOffClimberCommand;
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
import frc.robot.commands.tilt.CannonWomfMode;
import frc.robot.commands.tilt.StopTiltCommand;
import frc.robot.commands.womf.ActiveColorCommand;
import frc.robot.commands.womf.InactiveColorCommand;
import frc.robot.subsystems.CannonTiltSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivingSubsystem;
import frc.robot.subsystems.IntakeAndShootSubsystem;
import frc.robot.subsystems.PnuematicSubsytem;
import frc.robot.subsystems.WomfSubsystem;


public class RobotContainer {

    //public XboxController1 controller1;
    XboxController controller2 = new XboxController(Constants.XBOX_CONTROLLER_2_PORT);
    XboxController controller1 = new XboxController(Constants.XBOX_CONTROLLER_1_PORT);
   
    DrivingSubsystem drivingSubsystem = new DrivingSubsystem();
    ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    PnuematicSubsytem pnuematicSubsytem = new PnuematicSubsytem();
    IntakeAndShootSubsystem intakeAndShootSubsystem = new IntakeAndShootSubsystem();
    WomfSubsystem womfSubsystem = new WomfSubsystem();
    CannonTiltSubsystem cannonTiltSubsystem = new CannonTiltSubsystem();
    public RobotContainer(){
        configureButtonBindings();

        drivingSubsystem.setDefaultCommand(
            new DirectDriveCommand(
                drivingSubsystem,
                () -> getLeftDrive(),
                () -> getRightDrive()));
       
    }

    public double getRightDrive() {
        return controller1.getY(GenericHID.Hand.kRight);
		// return Math.abs(n) < 0.1 ? 0 : n;
    }
    public double getLeftDrive() {
		return controller1.getY(GenericHID.Hand.kLeft);
		// return Math.abs(n) < 0.1 ? 0 : n;
	}


    private void configureButtonBindings() {
        //womfButton, climberButton, cannonShoot, cannonIntake, climbModeCannon, cannonTiltIntake, cannonTiltShoot
        final JoystickButton cannonTiltIntake = new JoystickButton(controller2, XboxController.Button.kB.value);
        final JoystickButton womfButton = new JoystickButton(controller2, XboxController.Button.kX.value);
        final JoystickButton climberButton = new JoystickButton(controller1, XboxController.Button.kBumperRight.value);
        final JoystickButton cannonShoot = new JoystickButton(controller2,  XboxController.Button.kBumperRight.value);
        final JoystickButton climbModeCannon = new JoystickButton(controller1,  XboxController.Button.kBumperLeft.value);
        final JoystickButton cannonTiltShoot = new JoystickButton(controller2,  XboxController.Button.kA.value);
        final JoystickButton cannonIntake= new JoystickButton(controller2,  XboxController.Button.kBumperLeft.value);
        final JoystickButton compressorButton = new JoystickButton(controller2,  XboxController.Button.kY.value);


        final JoystickButton indexerShoot= new JoystickButton(controller1,  XboxController.Button.kB.value);
        final JoystickButton indexerIntake= new JoystickButton(controller1,  XboxController.Button.kA.value);

        climberButton.whenPressed(new ClimberCommand(climberSubsystem));
        climberButton.whenReleased(new TurnOffClimberCommand(climberSubsystem));
        womfButton.whenPressed(new ActiveColorCommand(womfSubsystem).alongWith(new CannonWomfMode(cannonTiltSubsystem)));
        cannonTiltIntake.whenPressed(new CannonIntakeMode(cannonTiltSubsystem));
        cannonTiltIntake.whenReleased(new StopTiltCommand(cannonTiltSubsystem));
        climbModeCannon.whenPressed(new CannonClimbMode(cannonTiltSubsystem));
        climbModeCannon.whenReleased(new StopTiltCommand(cannonTiltSubsystem));
        womfButton.whenReleased(new InactiveColorCommand(womfSubsystem).alongWith(new StopTiltCommand(cannonTiltSubsystem)));
        cannonTiltShoot.whenPressed(new CannonShootMode(cannonTiltSubsystem));
        cannonTiltShoot.whenReleased(new StopTiltCommand(cannonTiltSubsystem));
        cannonShoot.whenPressed(new ShooterOnCommand(intakeAndShootSubsystem));
        cannonShoot.whenReleased(new IntakeOffCommand(intakeAndShootSubsystem));
        cannonIntake.whenPressed(new IntakeCommand(intakeAndShootSubsystem));
        cannonIntake.whenReleased(new StopTiltCommand(cannonTiltSubsystem).alongWith(new IntakeOffCommand(intakeAndShootSubsystem)));
        indexerIntake.whenPressed(new IndexIntakeCommand(intakeAndShootSubsystem));
        indexerIntake.whenReleased(new StopIndexCommand(intakeAndShootSubsystem));
        indexerShoot.whenPressed(new IndexShooterCommand(intakeAndShootSubsystem));
        indexerShoot.whenReleased(new StopIndexCommand(intakeAndShootSubsystem));
        compressorButton.whenPressed(new StartCompressorCommand(pnuematicSubsytem));
        compressorButton.whenReleased(new StopCompressorCommand(pnuematicSubsytem));

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