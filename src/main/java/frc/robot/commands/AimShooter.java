// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ShooterPivot;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class AimShooter extends Command {
//   ShooterPivot s_ShooterPivot;
//   double setpoint;
//   /** Creates a new AimShooter. */
//   public AimShooter(ShooterPivot s_ShooterPivot) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.s_ShooterPivot = s_ShooterPivot;
//     //this.setpoint = setpoint;

//     addRequirements(s_ShooterPivot);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     s_ShooterPivot.setShooterAngle(s_ShooterPivot.getShooterAngle());
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     s_ShooterPivot.setShooterAngle(s_ShooterPivot.getShooterAngle());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
    
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
