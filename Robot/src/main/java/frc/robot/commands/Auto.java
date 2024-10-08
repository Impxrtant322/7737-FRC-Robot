// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto extends SequentialCommandGroup {
  SwerveSubsystem drivebase = new SwerveSubsystem();
  /** Creates a new Auto. */
  public Auto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Command driveFieldOrientedAngularVelocityStart = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(.1,0),
        () -> -MathUtil.applyDeadband(0,0),
        () -> MathUtil.applyDeadband(0,0));

    Command driveFieldOrientedAngularVelocityStop = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(0,0),
        () -> -MathUtil.applyDeadband(0,0),
        () -> MathUtil.applyDeadband(0,0));

    addCommands(driveFieldOrientedAngularVelocityStart, new wait(2), driveFieldOrientedAngularVelocityStop);
  }
}
