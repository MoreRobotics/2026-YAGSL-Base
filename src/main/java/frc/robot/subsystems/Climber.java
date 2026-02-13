// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Climber extends SubsystemBase {

//   private int servoID = 0;
//   private int servoIn = 0;
//   private int servoMid = 90;
//   private int servoOut = 0;

//   private Servo servo;
//   /** Creates a new Climber. */
//   public Climber() {
//     servo = new Servo(servoID);

//     servo.setBoundsMicroseconds(2500, 0, 1500, 0, 500);
//   }

//   public void setServo(int setpoint)
//   {
//     servo.setAngle(setpoint);
//   }
//   public int getServoIn()
//   {
//     return servoIn;
//   }
//   public int getServoMid()
//   {
//     return servoMid;
//   }
//   public int getServoOut()
//   {
//     return servoOut;
//   }

//   public double getServoPosition()
//   {
//     return servo.getAngle();
//   }


//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     SmartDashboard.putNumber("Servo Pulse Angle", getServoPosition());
//   }
// }
