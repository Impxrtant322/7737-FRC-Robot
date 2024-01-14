// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.path.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    private final XboxController mController = new XboxController(OperatorConstants.kDriverControllerPort);
    private final Swerve mSwerve = new Swerve();

    private final ManualDrive mManualDriveCommand = new ManualDrive(mSwerve, mController);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private PIDController mXController = new PIDController(
        SwerveConstants.kPathingX_kP, 
        SwerveConstants.kPathingX_kI, 
        SwerveConstants.kPathingX_kD
    );

    private PIDController mYController = new PIDController(
      SwerveConstants.kPathingY_kP, 
      SwerveConstants.kPathingY_kI, 
      SwerveConstants.kPathingY_kD
  );

  private PIDController mThetaController = new PIDController(
        SwerveConstants.kPathingTheta_kP, 
        SwerveConstants.kPathingTheta_kI, 
        SwerveConstants.kPathingTheta_kD
    );
/* 
    private PathPlannerTrajectory mTrajectory = PathPlanner.loadPath(
      "New Path", 
      SwerveConstants.kMaxVelocityMetersPerSecond, 
      SwerveConstants.kMaxAccelerationMetersPerSecond
  );*/
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    mSwerve.setDefaultCommand(mManualDriveCommand);
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    /* 
    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
            mTrajectory, 
            mSwerve::getPose, 
            SwerveConstants.kSwerveKinematics, 
            mXController, 
            mYController, 
            mThetaController, 
            mSwerve::setModuleStates, 
            mSwerve
        );

        return command.andThen(() -> mSwerve.drive(0, 0, 0, false));*/
        return null;
  }
}
