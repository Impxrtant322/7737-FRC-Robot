// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class Drive extends Command {

  SwerveSubsystem drivebase;
  DoubleSupplier LeftY;
  DoubleSupplier LeftX;
  DoubleSupplier rawAxis;
  boolean invertControl;
  /** Creates a new Drive. */
  public Drive(DoubleSupplier LeftY, DoubleSupplier LeftX, DoubleSupplier rawAxis, boolean invertControl, SwerveSubsystem drivebase) {
    this.LeftY = LeftY;
    this.LeftX = LeftX;
    this.rawAxis = rawAxis;
    this.invertControl = invertControl;
    this.drivebase = drivebase;

    addRequirements(this.drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(invertControl == false) {
    drivebase.driveCommand(
        () -> MathUtil.applyDeadband(LeftY.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(LeftX.getAsDouble(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(rawAxis.getAsDouble(), .05));
    } else if (invertControl == true) {
      drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(LeftY.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(LeftX.getAsDouble(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(rawAxis.getAsDouble(), .05));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.driveCommand(
        () -> MathUtil.applyDeadband(0, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(0, OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(0, .05));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
