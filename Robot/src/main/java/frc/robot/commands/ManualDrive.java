// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Swerve;

public class ManualDrive extends Command {
    private final Swerve mSwerve;
    private final XboxController mController;
  /** Creates a new ManualDrive. */
  public ManualDrive(Swerve drive, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerve = drive;
    mController = controller;

    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drives with XSpeed, YSpeed, zSpeed
    // True/false for field-oriented driving
    mSwerve.drive(mController.getLeftY(), mController.getLeftX(), mController.getRightX(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
