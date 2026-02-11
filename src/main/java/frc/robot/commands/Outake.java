// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HotDog;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Outake extends Command {
  Intake s_Intake;
  HotDog s_HotDog;
  /** Creates a new Outake. */
  public Outake(Intake s_Intake, HotDog s_HotDog) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    this.s_HotDog = s_HotDog;
    addRequirements(s_HotDog,s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Intake.setIntakeSpeed(s_Intake.getOutakeSpeed());
    s_HotDog.setHotDogSpeed(s_HotDog.getReverseHotDogSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.setIntakeSpeed(0);
    s_HotDog.setHotDogSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
