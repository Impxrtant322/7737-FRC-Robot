// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class emergencyStop extends Command {
  /** Creates a new emergencyStop. */
  IntakeSubsystem IntakeSubsystem;
  LauncherSubsystem LauncherSubsystem;
  public emergencyStop(IntakeSubsystem IntakeSubsystem, LauncherSubsystem LauncherSubsystem) {
    this.IntakeSubsystem = IntakeSubsystem;
    this.LauncherSubsystem = LauncherSubsystem;
    addRequirements(this.IntakeSubsystem, this.LauncherSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSubsystem.emergencyStop();
    LauncherSubsystem.emergencyStop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
