// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.Eyes;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Shooter extends SubsystemBase {
  private final Eyes s_Eyes;

  // Shooter Constants
  private int topLeftShooterID = 23;
  private int topRightShooterID = 25;

  private int bottomLeftShooterID = 14;
  private int bottomRightShooterID = 24;

  private double kP = .85 / 2;
  private double kI = 0;
  private double kD = 0;
  private double kV = 0.1;
  private double currentLimit = 40;
  private double speedOffset = 1;

  private double shooterSpeed = -83*.675;
  public boolean readyToShoot = false;

  private double targetSpeed;



  // Motor Classes
  private TalonFX m_TopLeftShooter;
  private TalonFX m_TopRightShooter;

  private TalonFX m_BottomLeftShooter;
  private TalonFX m_BottomRightShooter;

  private Follower m_LeftFollower;
  private Follower m_RightFollower;

  // YAMS mechanisms - one FlyWheel per side, each wrapping that side's top (leader) TalonFX.
  // Bottom motors stay plain CTRE Followers, untouched by YAMS, exactly as before.
  private FlyWheel m_LeftFlyWheel;
  private FlyWheel m_RightFlyWheel;

  /** Creates a new Shooter. */
  public Shooter(Eyes s_Eyes) {
    this.s_Eyes = s_Eyes;

    m_TopLeftShooter = new TalonFX(topLeftShooterID);
    m_TopRightShooter = new TalonFX(topRightShooterID);
    m_BottomLeftShooter = new TalonFX(bottomLeftShooterID);
    m_BottomRightShooter = new TalonFX(bottomRightShooterID);

    m_LeftFollower = new Follower(bottomLeftShooterID, MotorAlignmentValue.Aligned);
    m_RightFollower = new Follower(bottomRightShooterID, MotorAlignmentValue.Aligned);

    m_BottomLeftShooter.setControl(m_LeftFollower.withLeaderID(topLeftShooterID));
    m_BottomRightShooter.setControl(m_RightFollower.withLeaderID(topRightShooterID));

    // NOTE (pilot migration, see plan): the real robot's SensorToMechanismRatio was previously
    // set to 0, which is an unset/placeholder value (a real 0 gear ratio is meaningless) rather
    // than an intentional configuration - preserved here as a neutral 1:1 ratio via
    // SmartMotorControllerConfig's default gearing instead of reproducing what would likely be a
    // divide-by-zero.
    // TODO: motion-magic acceleration ramping (previously MotionMagicAcceleration = 52.84) was not
    // ported in this pilot migration - closed-loop velocity control works, but the real robot's
    // smooth ramp-to-target-speed feel needs to be re-added via
    // TalonFXWrapper#setMotionProfileMaxAcceleration once the units/value are re-validated.
    SmartMotorControllerConfig sharedConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(kP, kI, kD)
        .withFeedforward(new SimpleMotorFeedforward(0, kV, 0))
        .withGearing(MechanismGearing.kOne)
        .withStatorCurrentLimit(Amps.of(currentLimit))
        .withIdleMode(MotorMode.COAST)
        // Placeholder flywheel mass for simulation physics only - not a measured value, tune once
        // the real shooter wheel's mass is known. See SwerveSubsystem's maple-sim module config
        // comments for the same "documented placeholder, not a correctness issue" pattern.
        .withMomentOfInertia(Inches.of(2), Kilograms.of(0.5))
        .withTelemetry("Shooter", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    SmartMotorController leftMotor = new TalonFXWrapper(m_TopLeftShooter, DCMotor.getKrakenX60(1), sharedConfig.clone());
    SmartMotorController rightMotor = new TalonFXWrapper(m_TopRightShooter, DCMotor.getKrakenX60(1), sharedConfig.clone());

    // Placeholder flywheel diameter for simulation physics only - not a measured value.
    FlyWheelConfig flyWheelConfig = new FlyWheelConfig().withDiameter(Inches.of(4));
    m_LeftFlyWheel = new FlyWheel(flyWheelConfig.clone(), leftMotor);
    m_RightFlyWheel = new FlyWheel(flyWheelConfig.clone(), rightMotor);
  }

  public void setShooterSpeed(double speed)
  {
    targetSpeed = speed * speedOffset;
    SmartDashboard.putNumber("Shooter Set Speed", speed);
    m_LeftFlyWheel.getMotor().setVelocity(RotationsPerSecond.of(-speed * speedOffset));
    m_RightFlyWheel.getMotor().setVelocity(RotationsPerSecond.of(speed * speedOffset));
  }

  public void setShooterVoltage(double voltage)
  {
    m_LeftFlyWheel.getMotor().setVoltage(Volts.of(-voltage));
    m_RightFlyWheel.getMotor().setVoltage(Volts.of(voltage));
  }

  public double getCurrentSpeed() {
    return m_TopLeftShooter.getVelocity().getValueAsDouble();
  }

  /**
   * Last commanded flywheel speed (rotations/sec), as set by {@link #setShooterSpeed}. Used in
   * simulation to approximate Fuel piece launch velocity.
   */
  public double getTargetSpeed()
  {
    return targetSpeed;
  }

  public double getShooterSpeed()
  {
    double speed;
    speed = -4.15*s_Eyes.getTargetDistance()
    -32.1625;

    if(speed > -39.42)
    {
      return -39.42;
    }
    else if(speed < -56.025)
    {
      return -56.025;
    }
    else{
      return speed*1.03;
    }

  }

  public boolean isReadyToShoot()
    {
      if(m_TopRightShooter.getVelocity().getValueAsDouble() < (targetSpeed * .9))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    public void increaseShooterSpeed()
    {
      speedOffset += 0.01;
    }

    public void decreaseShooterSpeed()
    {
      speedOffset -= 0.01;
    }

    public void resetShooterSpeed()
    {
      speedOffset = 1;
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_LeftFlyWheel.updateTelemetry();
    m_RightFlyWheel.updateTelemetry();

    // Logging Variables
    SmartDashboard.putNumber("Top Left Shooter Motor Speed", m_TopLeftShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Bottom Left Shooter Motor Speed", m_BottomLeftShooter.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Top Right Shooter Motor Speed", m_TopRightShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Bottom Right Shooter Motor Speed", m_BottomRightShooter.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Top Left Shooter Motor Current", m_TopLeftShooter.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Bottom Left Shooter Motor Current", m_BottomLeftShooter.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Top Right Shooter Motor Current", m_TopRightShooter.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Bottom Right Shooter Motor Current", m_BottomRightShooter.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putBoolean("Ready To Shoot", isReadyToShoot());

  }

  @Override
  public void simulationPeriodic() {
    m_LeftFlyWheel.simIterate();
    m_RightFlyWheel.simIterate();
  }
}
