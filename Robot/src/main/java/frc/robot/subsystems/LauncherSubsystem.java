// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class LauncherSubsystem extends SubsystemBase {
  private double power = 1;
  private String toggle = "off";
  private final CANSparkMax motor1 = new CANSparkMax(21, MotorType.kBrushless);
  private final CANSparkMax motor2 = new CANSparkMax(22, MotorType.kBrushless);
  private final CANSparkMax motor3 = new CANSparkMax(23, MotorType.kBrushless);
  private final CANSparkMax motor4 = new CANSparkMax(24, MotorType.kBrushless);

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    motor1.set(0);
    motor2.set(0);
    motor3.set(0);
    motor4.set(0);
    motor2.follow(motor1);
    motor4.follow(motor3);
  }

  public void launch() {
    motor1.set(power);
    motor3.set(-power);
  }

  public void stop() {
    motor1.set(0);
    motor3.set(0);
  }

  public void load() {
    motor1.set(0);
    motor3.set(0);
    try {
      Thread.sleep(120);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  public void toggle() {
    if(toggle == "off") {
      toggle = "on";
      motor1.set(power);
      motor3.set(-power);
    } else if (toggle == "on") {
      toggle = "off";
      motor1.set(0);
      motor3.set(0);
    }
  }

  public void intake() {
    motor1.set(0);
    motor3.set(0);
  }

  public void emergencyStop() {
    motor1.set(0);
    motor3.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
