package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private CANSparkMax mRotor;
    private CANSparkMax mThrottle;

    // throttle encoder
    private RelativeEncoder mThrottleEncoder;

    // rotor encoder
    private Canandcoder mRotorEncoder;

    // rotor PID controller
    private PIDController mRotorPID;

    /**
     * SwerveModule
     *
     * @param throttleID CAN ID of throttle 
     * @param rotorID CAN ID of rotor 
     * @param rotorEncoderID CAN ID of rotor encoder
     * @param rotorOffsetAngleDeg rotor encoder 
     */
    public SwerveModule(int throttleID, int rotorID, int rotorEncoderID, double rotorOffsetAngleDeg)
    {
        // throttle & encoder
        mThrottle = new CANSparkMax(throttleID, MotorType.kBrushless);
        mThrottleEncoder = mThrottle.getEncoder();

        //  rotor 
        mRotor = new CANSparkMax(rotorID, MotorType.kBrushless);

        //  rotor absolute encoder
        mRotorEncoder = new Canandcoder(rotorEncoderID);

        // 
        mThrottle.restoreFactoryDefaults();
        mRotor.restoreFactoryDefaults();
        mRotorEncoder.resetFactoryDefaults(true);

        //  rotor 
        mRotor.setInverted(SwerveConstants.kRotorMotorInversion);
        mRotor.enableVoltageCompensation(SwerveConstants.kVoltageCompensation);
        mRotor.setIdleMode(IdleMode.kBrake);

        //rotor encoder
        mRotorEncoder.setAbsPosition(0);

        //  rotor
        mRotorPID = new PIDController(
            SwerveConstants.kRotor_kP,
            SwerveConstants.kRotor_kI,
            SwerveConstants.kRotor_kD
        );

        // ContinuousInput  min  max
        mRotorPID.enableContinuousInput(-180, 180);

        //  throttle
        mThrottle.enableVoltageCompensation(SwerveConstants.kVoltageCompensation);
        mThrottle.setIdleMode(IdleMode.kBrake);

        // throttle encoder
        mThrottleEncoder.setVelocityConversionFactor(
            SwerveConstants.kThrottleVelocityConversionFactor
        );
        mThrottleEncoder.setPositionConversionFactor(
            SwerveConstants.kThrottlePositionConversionFactor
        );
    }

    /**
     * Return current state of module
     * 
     * @return module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            mThrottleEncoder.getVelocity(),
            Rotation2d.fromDegrees(mRotorEncoder.getAbsPosition())
        );
    }
    
    /**
     * Return current position of module
     * 
     * @return module position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            mThrottleEncoder.getPosition(), 
            Rotation2d.fromDegrees(mRotorEncoder.getAbsPosition())
        );
    }

    /**
     * Set module state
     * 
     * @param state module state 
     */
    public void setState(SwerveModuleState state) {
        // 90
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
        
        // PID 
        double rotorOutput = mRotorPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        mRotor.set(rotorOutput);
        mThrottle.set(optimizedState.speedMetersPerSecond);
    }
}