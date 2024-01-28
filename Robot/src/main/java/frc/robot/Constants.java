// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  
  public final class Controller {

		public static final int m_controller = 0;

		public final class Joystick {

			public static final int m_leftStickY = 1;
			public static final int m_leftStickX = 0;
			public static final int m_rightStickX = 4;
			// public static final int m_rightTrigger = 3;



			public static final int m_leftTrigger = 2;
			public static final int m_rightTrigger = 3;
		}

		public final class Buttons {

			public static final int m_aButton = 1;
			public static final int m_bButton = 2;
			public static final int m_xButton = 3;
			public static final int m_yButton = 4;
		}

		public final class Bumpers {

			public static final int m_lBumper = 5;
			public static final int m_rBumper = 6;
		}
	}
}
