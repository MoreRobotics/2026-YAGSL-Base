// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private int servoID = 0;
  private int servoIn = 0;
  private int servoMid = 90;
  private int servoOut = 0;

  private double climberDown = 0;
  private double climberUp = 0;
  private double climberHalfWay = 0;

  private int climberID = 0;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double forwardLimit = 0;
  private double reverseLimit = 0;
  private double gearRatio = 0;
  private double currentLimit = 0;
  private double acceleration = 0;
  private double velocity = 0;
  private double tolerance = 0;
  private double target = 0;

  private Servo servo;
  private TalonFX m_Climber;
  private MotionMagicVoltage m_Request;
  private TalonFXConfiguration configs;
  /** Creates a new Climber. */
  public Climber() {
    servo = new Servo(servoID);
    m_Climber = new TalonFX(climberID);
    m_Request = new MotionMagicVoltage(0).withSlot(0);

    servo.setBoundsMicroseconds(2500, 0, 1500, 0, 500);

    configs =  new TalonFXConfiguration();
     configs.Slot0.kP = kP;
    configs.Slot0.kI = kI;
    configs.Slot0.kD = kD;
    configs.MotionMagic.MotionMagicAcceleration = acceleration;
    configs.MotionMagic.MotionMagicCruiseVelocity = velocity;
    // configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    // configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
    configs.Feedback.SensorToMechanismRatio = gearRatio;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_Climber.getConfigurator().apply(configs);
  }


  public void setClimberPosition(double setpoint)
  {
    target = setpoint;
    m_Climber.setControl(m_Request.withPosition(setpoint));
  }

  public double getClimberDownPose()
  {
    return climberDown;
  }

  public double getClimberUpPose()
  {
    return climberUp;
  }

  public double getClimberHalfWayPose()
  {
    return climberHalfWay;
  }

   public boolean atPosition()
  {
    return (Math.abs(getClimberPosition() - target) < tolerance);
  }

  public double getClimberPosition()
  {
    return m_Climber.getPosition().getValueAsDouble();
  }

  public void setServo(int setpoint)
  {
    servo.setAngle(setpoint);
  }
  public int getServoIn()
  {
    return servoIn;
  }
  public int getServoMid()
  {
    return servoMid;
  }
  public int getServoOut()
  {
    return servoOut;
  }

  public double getServoPosition()
  {
    return servo.getAngle();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Servo Pulse Angle", getServoPosition());
  }
}
