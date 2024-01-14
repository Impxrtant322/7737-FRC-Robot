// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.SwerveConstants;
import com.kauailabs.navx.frc.AHRS;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */

  private final AHRS mImu = new AHRS(SPI.Port.kMXP);

  private final SwerveModule mLeftFrontModule, mRightFrontModule, mLeftRearModule, mRightRearModule;
  private SwerveDriveOdometry mOdometry;

  public Swerve() {
    // Instantiate swerve modules - each representing unique module on the robot
        mLeftFrontModule = new SwerveModule(
            SwerveConstants.kLeftFrontThrottleID, 
            SwerveConstants.kLeftFrontRotorID, 
            SwerveConstants.kLeftFrontRotorEncoderID, 
            SwerveConstants.kLeftFrontRotorOffset
        );

        mRightFrontModule = new SwerveModule(
            SwerveConstants.kRightFrontThrottleID, 
            SwerveConstants.kRightFrontRotorID, 
            SwerveConstants.kRightFrontRotorEncoderID, 
            SwerveConstants.kRightFrontRotorOffset
        );

        mLeftRearModule = new SwerveModule(
            SwerveConstants.kLeftRearThrottleID, 
            SwerveConstants.kLeftRearRotorID, 
            SwerveConstants.kLeftRearRotorEncoderID, 
            SwerveConstants.kLeftRearRotorOffset
        );

        mRightRearModule = new SwerveModule(
            SwerveConstants.kRightRearThrottleID, 
            SwerveConstants.kRightRearRotorID, 
            SwerveConstants.kRightRearRotorEncoderID, 
            SwerveConstants.kRightRearRotorOffset
        );

    //Instantiate odometry, for tracking position
    mOdometry = new SwerveDriveOdometry(SwerveConstants.kSwerveKinematics, mImu.getRotation2d(), getModulePositions());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mOdometry.update(
            mImu.getRotation2d(), 
            getModulePositions()
        );
    }

    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
      SwerveModuleState[] states = null;
      if(fieldOriented) {
          states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
              // IMU used for field oriented control
              // IMU Field Oriented Control
              ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, mImu.getRotation2d())
          );
      } else {
          states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
              new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
          );
      }
      setModuleStates(states);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
        mLeftFrontModule.getState(), 
        mRightFrontModule.getState(), 
        mLeftRearModule.getState(), 
        mRightRearModule.getState()
    };
}

public SwerveModulePosition[] getModulePositions() {
  return new SwerveModulePosition[] {
      mLeftFrontModule.getPosition(), 
      mRightFrontModule.getPosition(), 
      mLeftRearModule.getPosition(), 
      mRightRearModule.getPosition()
  };
}

public void setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
  mLeftFrontModule.setState(desiredStates[0]);
  mRightFrontModule.setState(desiredStates[1]);
  mLeftRearModule.setState(desiredStates[2]);
  mRightRearModule.setState(desiredStates[3]);
}

public Pose2d getPose() {
  return mOdometry.getPoseMeters();
}

public void setPose(Pose2d pose) {
  mOdometry.resetPosition(mImu.getRotation2d(), getModulePositions(), pose);
}
}

