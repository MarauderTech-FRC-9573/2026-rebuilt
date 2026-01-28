// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.util.Objects;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
   public SwerveController swerveController;
   File directory = new File(Filesystem.getDeployDirectory(), "swerve");
   SwerveDrive swerveDrive;
   private final boolean visionDriveTest = false;
   private final Field2d m_field = new Field2d();
   public double translationSpeed = 1.0;

   public SwerveSubsystem() {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

      try {
         boolean blueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;
         Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Units.Meter.of(1.0), Units.Meter.of(4.0)), Rotation2d.fromDegrees(0.0)) : new Pose2d(new Translation2d(Units.Meter.of(16.0), Units.Meter.of(4.0)), Rotation2d.fromDegrees(180.0));
         this.swerveDrive = (new SwerveParser(this.directory)).createSwerveDrive(Constants.SwerveConstants.speedMax, startingPose);
      } catch (Exception var5) {
         throw new RuntimeException(var5);
      }

      SmartDashboard.putData("Field", this.m_field);
      this.swerveController = this.swerveDrive.swerveController;
      boolean enableFeedforward = true;

      try {
         RobotConfig config = RobotConfig.fromGUISettings();
         AutoBuilder.configure(this::getPose, this::resetOdometry, this::getRobotVelocity, (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
               this.swerveDrive.drive(speedsRobotRelative, this.swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative), moduleFeedForwards.linearForces());
            } else {
               this.swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }

         }, new PPHolonomicDriveController(new PIDConstants(10.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.2)), config, () -> {
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
               return alliance.get() == Alliance.Red;
            } else {
               return false;
            }
         }, new Subsystem[]{this});
      } catch (Exception var4) {
         System.out.println(var4);
      }

      PathfindingCommand.warmupCommand().schedule();
   }

   public void resetOdometry(Pose2d initialHolonomicPose) {
      this.swerveDrive.resetOdometry(initialHolonomicPose);
   }

   public ChassisSpeeds getRobotVelocity() {
      return this.swerveDrive.getRobotVelocity();
   }

   // public void setupPhotonVision() {
   //    SwerveDrive var10002 = this.swerveDrive;
   //    Objects.requireNonNull(var10002);
   //    new Vision(var10002::getPose, this.swerveDrive.field);
   //    System.out.println("Photon Vision Setup");
   // }

   public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
      return this.run(() -> {
         Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), this.translationSpeed);
         SmartDashboard.putNumber("headingX", headingX.getAsDouble());
         SmartDashboard.putNumber("headingY", headingY.getAsDouble());
         SmartDashboard.putNumber("setpoint", this.swerveDrive.swerveController.lastAngleScalar);
         this.driveFieldOriented(this.swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX.getAsDouble(), headingY.getAsDouble(), this.swerveDrive.getOdometryHeading().getRadians(), this.swerveDrive.getMaximumChassisVelocity()));
      });
   }

   public double changeSpeed(double newSpeed) {
      this.translationSpeed = newSpeed;
      return newSpeed;
   }

   public void getIMU() {
   }

   public boolean exampleCondition() {
      return false;
   }

   public void periodic() {
      SmartDashboard.putNumber("Drivetrain Speed", this.translationSpeed);
      SmartDashboard.putNumber("Drivetrain Yaw", this.swerveDrive.getYaw().getDegrees());
      this.m_field.setRobotPose(this.swerveDrive.getPose());
   }

   public void simulationPeriodic() {
   }

   public SwerveDrive getSwerveDrive() {
      return this.swerveDrive;
   }

   public void driveFieldOriented(ChassisSpeeds velocity) {
      this.swerveDrive.driveFieldOriented(velocity);
   }

   public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
      return this.run(() -> {
         this.swerveDrive.driveFieldOriented((ChassisSpeeds)velocity.get());
      });
   }

   public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
      Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
      return this.swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), angle.getRadians(), this.getHeading().getRadians(), this.swerveDrive.getMaximumChassisVelocity());
   }

   public Rotation2d getHeading() {
      return this.swerveDrive.getPose().getRotation();
   }

   public Pose2d getPose() {
      return this.swerveDrive.getPose();
   }
}