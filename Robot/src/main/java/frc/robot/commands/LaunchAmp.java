// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class LaunchAmp extends Command {
  /** Creates a new Launch. */
  IntakeSubsystem IntakeSubsystem;
  LauncherSubsystem LauncherSubsystem;
  Timer time = new Timer();
  public LaunchAmp(IntakeSubsystem IntakeSubsystem, LauncherSubsystem LauncherSubsystem) {
    this.IntakeSubsystem = IntakeSubsystem;
    this.LauncherSubsystem = LauncherSubsystem;

    addRequirements(this.IntakeSubsystem, this.LauncherSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(time.get() > 0 && time.get() <= .5) {
      LauncherSubsystem.output();
    }
    if(time.get() > .5 && time.get() <= .7) {
      LauncherSubsystem.stop();
    }
    if(time.get() > .7 && time.get() < 2) {
      LauncherSubsystem.launchAmp();
    }
    if(time.get() >= 2) {
      IntakeSubsystem.launchAmp();
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
    return time.get() >= 3;
  }
}
