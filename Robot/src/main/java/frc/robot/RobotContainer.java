// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.commands.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.Intake;
import frc.robot.commands.Launcher;
import frc.robot.commands.emergencyStop;
import frc.robot.commands.Launch;
import frc.robot.commands.LaunchAmp;
import frc.robot.commands.LaunchAutoSpeaker;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Auto;

import java.io.File;

import com.pathplanner.lib.path.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import swervelib.SwerveDrive;
import swervelib.math.*;
import swervelib.parser.SwerveParser;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

    private final XboxController driverXbox = new XboxController(OperatorConstants.kDriverControllerPort);

	public Trigger xButtonDriver = new JoystickButton(driverXbox, Constants.Controller.Buttons.m_xButton);
	public Trigger yButtonDriver = new JoystickButton(driverXbox, Constants.Controller.Buttons.m_yButton);
	public Trigger bButtonDriver = new JoystickButton(driverXbox, Constants.Controller.Buttons.m_bButton);
	public Trigger aButtonDriver = new JoystickButton(driverXbox, Constants.Controller.Buttons.m_aButton);

	public Trigger rBumperDriver = new JoystickButton(driverXbox, Constants.Controller.Bumpers.m_rBumper);
	public Trigger lBumperDriver = new JoystickButton(driverXbox, Constants.Controller.Bumpers.m_lBumper);

	public POVButton UPDriver = new POVButton(driverXbox, 0);
	public POVButton DOWNDriver = new POVButton(driverXbox, 180);
	public POVButton LEFTDriver = new POVButton(driverXbox, 270);
	public POVButton RIGHTDriver = new POVButton(driverXbox, 90);

    SwerveSubsystem drivebase = new SwerveSubsystem();
    LauncherSubsystem LauncherSubsystem = new LauncherSubsystem();
    IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4), .05));

    

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    xButtonDriver.onTrue(new InstantCommand(drivebase::zeroGyro));
    aButtonDriver.onTrue(new Intake(IntakeSubsystem, LauncherSubsystem));
    yButtonDriver.onTrue(new Launch(IntakeSubsystem, LauncherSubsystem));
    bButtonDriver.onTrue(new LaunchAmp(IntakeSubsystem, LauncherSubsystem));
    DOWNDriver.onTrue(new emergencyStop(IntakeSubsystem, LauncherSubsystem));

    //aButtonDriver.onTrue(new Launcher(LauncherSubsystem, "toggle"));
    //bButtonDriver.onTrue(new Intake(IntakeSubsystem, "toggle"));
    //bButtonDriver.onTrue(new Launcher(LauncherSubsystem, "load"));

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
    return new LaunchAutoSpeaker(IntakeSubsystem, LauncherSubsystem);
    //return new Auto();
  }
}
