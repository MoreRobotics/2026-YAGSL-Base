// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HotDog;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunHotDog extends Command {
  HotDog s_HotDog;
  /** Creates a new RunHotDog. */
  public RunHotDog(HotDog s_HotDog) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_HotDog = s_HotDog;
    addRequirements(s_HotDog);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_HotDog.setHotDogSpeed(s_HotDog.getHotDogSpeed());
    s_HotDog.setIndexerSpeed(s_HotDog.getIndexerSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_HotDog.setHotDogSpeed(0);
    s_HotDog.setIndexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
