// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class Feeder extends SubsystemBase {

    private TalonFX m_leftMotor;
    private TalonFX m_rightMotor;

    private MotionMagicVelocityVoltage m_velocityRequest;
    

    private double gearRatio = 3/1;
    private int leftMotorID = 0;
    private int rightMotorID = 0;


  /** Creates a new Shooter. */
  public Feeder() {
    m_leftMotor = new TalonFX(leftMotorID);
    m_rightMotor = new TalonFX(rightMotorID);

    m_velocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
  }

  public void setMotorSpeed( double leftMotorSpeed, double rightMotorSpeed) {

    m_leftMotor.setControl(m_velocityRequest.withVelocity(leftMotorSpeed));  
    m_rightMotor.setControl(m_velocityRequest.withVelocity(rightMotorSpeed));

  };

  public double getMotorSpeed ( double leftMotorSpeed, double rightMotorSpeed) {

    return m_leftMotor.getVelocity().getValueAsDouble();

  };


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
