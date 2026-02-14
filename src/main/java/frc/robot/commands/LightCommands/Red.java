package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;

public class Red extends Command {
    Lights s_Lights;
  /** Creates a new MoveIntake. */
    public Red(Lights s_Lights) {
        this.s_Lights = s_Lights;
        addRequirements(s_Lights);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    @Override
    public void initialize() {
        // Runs once when scheduled
        s_Lights.Defense();
    }

    @Override
    public void execute() {
        // Runs repeatedly while scheduled
    }

    @Override
    public void end(boolean interrupted) {
        // Runs once when the command ends
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
