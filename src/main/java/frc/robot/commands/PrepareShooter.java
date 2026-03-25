// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LightCommands.Aim;
import frc.robot.commands.LightCommands.Clear;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepareShooter extends Command {
  Shooter s_Shooter;
  Lights s_Lights;

  double shooterTolerance = 2.0;

  /** Creates a new Shoot. */
  public PrepareShooter(Shooter s_Shooter, Lights s_Lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Shooter = s_Shooter;
    this.s_Lights = s_Lights;
    addRequirements(s_Shooter, s_Lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Shooter.setShooterSpeed(s_Shooter.getShooterSpeed());

    if(Math.abs(s_Shooter.getShooterSpeed() - s_Shooter.getCurrentSpeed()) < shooterTolerance) {
      s_Lights.ReadyToFire();
    } else {
      s_Lights.Aim();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.setShooterSpeed(-5);
    s_Lights.ClearLights();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
