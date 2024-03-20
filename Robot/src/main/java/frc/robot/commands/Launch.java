// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class Launch extends Command {
  /** Creates a new Launch. */
  IntakeSubsystem IntakeSubsystem;
  LauncherSubsystem LauncherSubsystem;
  boolean isFinished = false;
  Timer time = new Timer();
  public Launch(IntakeSubsystem IntakeSubsystem, LauncherSubsystem LauncherSubsystem) {
    this.IntakeSubsystem = IntakeSubsystem;
    this.LauncherSubsystem = LauncherSubsystem;

    addRequirements(this.IntakeSubsystem, this.LauncherSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isFinished = false;
    LauncherSubsystem.launch();
    if(time.get() >= 1.25) {
      IntakeSubsystem.launch();
    }
    if(time.get() >= 2.25) {
      IntakeSubsystem.stop();
      LauncherSubsystem.stop();
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.stop();
    LauncherSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinished) {
      return true;
    }
    return false;
  }
}
