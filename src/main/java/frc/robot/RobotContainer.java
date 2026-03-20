// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmManualControl;
import frc.robot.commands.FeedFuel;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootFuel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.FuelConstants.FEEDER_MOTOR_SPEED;
import static frc.robot.Constants.FuelConstants.SHOOTER_MOTOR_SPEED;
import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;
import static frc.robot.Constants.ArmConstants.ARM_MAX_SPEED;

import java.io.File;
import swervelib.SwerveInputStream;
// import frc.robot.commands.LaunchSequence;
// import frc.robot.commands.SpinUp;
// import frc.robot.subsystems.OutakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController m_driverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  final         CommandXboxController m_operatorController = new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyroWithAlliance)
                                                    .andThen(Commands.none()));

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
                                                .andThen(drivebase.driveForward().withTimeout(1)));
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    if (autoChooser.getSelected() == null ) {
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    //Runs feeder 
    m_operatorController.a().whileTrue(new FeedFuel(feeder, FEEDER_MOTOR_SPEED));

    //Runs Shooter (orange roller)
    m_operatorController.b().whileTrue(new ShootFuel(shooter, SHOOTER_MOTOR_SPEED));

    //Runs feeder and Shooter simustaneouly 
    m_operatorController.x().whileTrue(new FeedFuel(feeder, FEEDER_MOTOR_SPEED)
                                      .alongWith(new ShootFuel(shooter, SHOOTER_MOTOR_SPEED)));

    m_operatorController.y().whileTrue(new IntakeCommand(intake, INTAKE_SPEED));

    m_operatorController.rightTrigger().whileTrue(new ArmManualControl(arm, ARM_MAX_SPEED));
    m_operatorController.leftTrigger().whileTrue(new ArmManualControl(arm, -ARM_MAX_SPEED));


    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);


    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      m_driverController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      m_driverController.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));



    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!

      m_driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.back().whileTrue(drivebase.centerModulesCommand());
      m_driverController.leftBumper().onTrue(Commands.none());
      m_driverController.rightBumper().onTrue(Commands.none());
    } else
    {
      m_driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.start().whileTrue(Commands.none());
      m_driverController.back().whileTrue(Commands.none());
      m_driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.rightBumper().onTrue(Commands.none());
    }

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
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  
}
