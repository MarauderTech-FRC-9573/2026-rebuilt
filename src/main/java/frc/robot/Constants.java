// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.15;
  }

   public static final class FuelConstants {
    //TODO: Change the motor IDs and actually test voltages.
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 6;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 5;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1;
  }
    public static class SwerveConstants {
    //The max Speed of the bot
    public static final double MAXSPEED = Units.feetToMeters(16.5);

    // Variables to change the speed of the bot
    public static final double CHANGE_SPEED_MAX = 2;
    public static final double CHANGE_SPEED_MIN = 0.5;
    public static final double CHANGE_SPEED_DEFAULT = 1;
  }

  public static class ArmConstants {

  }

  public static class IntakeConstants {
    public static final int Evasrage = 1;
    public static final int intakeMotorID = 0;
    public static Runnable defaultIntakeSpeed;
  }
}
