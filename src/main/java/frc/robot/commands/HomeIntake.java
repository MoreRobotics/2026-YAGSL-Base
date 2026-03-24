// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeIntake extends Command {
  Intake s_Intake;
  boolean finished = false;
  /** Creates a new HomeIntake. */
  public HomeIntake(Intake s_Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    s_Intake.homeIntakePivot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(s_Intake.getIntakePivotCurrent() >= 25)
    // {
    //   System.out.println("Working");
    //   // s_Intake.stopIntakePivot();
    //   s_Intake.setIntakePivotPosition(s_Intake.getIntakeOutPosition());
    //   finished = true;
    // }
    // else
    // {
    //   finished = false;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.stopIntakePivot();
    s_Intake.setIntakePivotPosition(s_Intake.getIntakeOutPosition() + 0.0131);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Intake.getIntakePivotCurrent() >= 25;
  }
}
