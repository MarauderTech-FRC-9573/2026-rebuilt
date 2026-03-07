// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.OutakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveInputStream;
import frc.robot.commands.Launch;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.SpinUp;
// import frc.robot.subsystems.OutakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final OutakeSubsystem outake = new OutakeSubsystem();

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatotController = 
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
     autoChooser = AutoBuilder.buildAutoChooser("Leave Auto");
      SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> m_driverController.getLeftY() * -1,
            () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(m_driverController::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

     SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
             .withControllerHeadingAxis(m_driverController::getRightX,
                     m_driverController::getRightY)
             .headingWhile(true);

        //Robot Relative needs testing
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
             .allianceRelativeControl(false);
 

    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
            () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1),
            () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
            () -> MathUtil.applyDeadband(m_driverController.getRightY(), 0.0),
            () -> MathUtil.applyDeadband(m_driverController.getRightX(), 0.0));

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //TODO: Ask strategy people/muhammad about button bindings. Add button Bindings.
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //m_driverController.a().onTrue(new LaunchSequence(outake));
    //m_driverController.b().whileTrue(new SpinUp(outake));
    //m_driverController.x().whileTrue(new Launch(outake));
  }

      

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // TODO: make new RunCommand to fix this idk 
    //m_driverController.b().whileTrue(new Command(() -> Outake.run(IntakeConstants.defaultIntakeSpeed)));


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
      public Command getAutomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
  
}
