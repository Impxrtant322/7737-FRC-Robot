// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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
  }
  
  public static class SwerveConstants {
    // Rotor IDs
    public static final int kLeftFrontRotorID = 0;
    public static final int kRightFrontRotorID = 0;
    public static final int kLeftRearRotorID = 0;
    public static final int kRightRearRotorID = 0;
 
    // Throttle IDs
    public static final int kLeftFrontThrottleID = 0;
    public static final int kRightFrontThrottleID = 0;
    public static final int kLeftRearThrottleID = 0;
    public static final int kRightRearThrottleID = 0;

    // Rotor Encoder IDs
    public static final int kLeftFrontRotorEncoderID = 0;
    public static final int kRightFrontRotorEncoderID = 0;
    public static final int kLeftRearRotorEncoderID = 0;
    public static final int kRightRearRotorEncoderID = 0;

    // Encoder Offsets
    public static final double kLeftFrontRotorOffset = -1; //* LEFT_FRONT_ANGLE;
    public static final double kRightFrontRotorOffset = -1; //* RIGHT_FRONT_ANGLE;
    public static final double kLeftRearRotorOffset = -1; //* LEFT_REAR_ANGLE;
    public static final double kRightRearRotorOffset = -1; //* RIGHT_REAR_ANGLE;

    //Inversion Booleans
    public static final boolean kRotorEncoderDirection = false;
    public static final boolean kRotorMotorInversion = false;

    // Swerve module order: front left, front right, rear left, rear right
    
    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      new Translation2d(1/2, 1/2),
      new Translation2d(1/2, -1/2),
      new Translation2d(-1/2, 1/2),
      new Translation2d(-1/2, -1/2)
    );
    /*new Translation2d(LENGTH/2, WIDTH/2),
      new Translation2d(LENGTH/2, -WIDTH/2),
      new Translation2d(-LENGTH/2, WIDTH/2),
      new Translation2d(-LENGTH/2, -WIDTH/2) */
    
    //Max speed and acceleration
    public static final double kMaxVelocityMetersPerSecond = 0.0;
    public static final double kMaxAccelerationMetersPerSecond = 0.0;

    //Wheel Diameter
    public static final double kWheelDiameterMeters = 0.0;
    
    // Throttle gear ratio
    public static final double kThrottleGearRatio = 0.0;

    //conversion factors
    //Position factor
    public static final double kThrottleVelocityConversionFactor =
      1/kThrottleGearRatio*kWheelDiameterMeters*Math.PI;


    //Velocity factor
    
    public static final double kThrottlePositionConversionFactor =
      1/kThrottleGearRatio/60*kWheelDiameterMeters*Math.PI;
      

    //Encoder stuff
    public static final double kRotor_kP = 0.0;
    public static final double kRotor_kI = 0.0;
    public static final double kRotor_kD = 0.0;

    //Voltage Compensation
    public static final double kVoltageCompensation = 12.0;

    // Pathing PID constants 
    public static final double kPathingX_kP = 0.1;
    public static final double kPathingX_kI = 0.0;
    public static final double kPathingX_kD = 0.0;

    public static final double kPathingY_kP = 0.1;
    public static final double kPathingY_kI = 0.0;
    public static final double kPathingY_kD = 0.0;

    public static final double kPathingTheta_kP = 0.1;
    public static final double kPathingTheta_kI = 0.0;
    public static final double kPathingTheta_kD = 0.0;
  }
}
