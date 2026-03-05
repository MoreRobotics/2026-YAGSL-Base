// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Feeder extends SubsystemBase {

    private TalonFX m_leftMotor;
    private TalonFX m_rightMotor;

    private MotionMagicVelocityVoltage m_velocityRequest;
    private TalonFXConfiguration configs;
    

    private int leftMotorID = 17;
    private int rightMotorID = 18;


    private double kP = .33;
    private double kI = 0;
    private double kD = 0;
    private double kV = 0.1;
    private double currentLimit = 70;
    private double acceleration = 50;
    private double leftFeederSpeed = -40;
    private double rightFeederSpeed = 5;


  /** Creates a new Shooter. */
  public Feeder() {
    m_leftMotor = new TalonFX(leftMotorID);
    m_rightMotor = new TalonFX(rightMotorID);

    m_velocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

    configs = new TalonFXConfiguration();
    configs.Slot0.kP = kP;
    configs.Slot0.kI = kI;
    configs.Slot0.kD = kD;
    configs.Slot0.kV = kV;
    configs.MotionMagic.MotionMagicAcceleration = acceleration;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = currentLimit;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_leftMotor.getConfigurator().apply(configs);
    m_rightMotor.getConfigurator().apply(configs);
  }

  public void setFeederSpeed( double leftMotorSpeed, double rightMotorSpeed) {

    m_leftMotor.setControl(m_velocityRequest.withVelocity(leftMotorSpeed));  
    // m_rightMotor.setControl(m_velocityRequest.withVelocity(rightMotorSpeed));

  };


  public double getLeftFeederSpeed()
  {
    return leftFeederSpeed;
  }

  public double getRightFeederSpeed()
  {
    return rightFeederSpeed;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // shooter.simIterate();
  }
}
