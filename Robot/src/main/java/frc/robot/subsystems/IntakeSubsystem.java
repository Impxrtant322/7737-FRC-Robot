// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

public class IntakeSubsystem extends SubsystemBase {
  private double power = -.5;
  private String toggle = "off";
  private final CANSparkMax motor1 = new CANSparkMax(31, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor1.getEncoder();
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor1.setIdleMode(IdleMode.kBrake);
  }

  public void intakeOn() {
    motor1.set(power);
  }

 public void intakeOff() {
  motor1.set(0);
 }

 public void toggle() {
  if(toggle == "off") {
    toggle = "on";
    intakeOn();
  } else if (toggle == "on") {
    toggle = "off";
    intakeOff();
  }
}

public void launch() {
  motor1.set(power);
}

public void launchAmp() {
  motor1.set(power);
}

public void stop() {
  motor1.set(0);
}

public double getEncoderVelocity() {
  return encoder.getVelocity();
}

public void intake() {
  intakeOn();
  intakeOff();
}

public void emergencyStop() {
  motor1.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
