// package frc.robot.commands.LightCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Lights;
// import frc.robot.subsystems.swervedrive.*;
// import frc.robot.subsystems.ShooterPivot;

// public class Aim extends Command {
//     Lights s_Lights;

//     double swerveTolerance = 2.0;
//     double hoodTolerance = 1.0;

//   /** Creates a new MoveIntake. */
//     public Aim(Lights s_Lights) {
//         this.s_Lights = s_Lights;

//         addRequirements(s_Lights);
//         // Use addRequirements() here to declare subsystem dependencies.
//     }

//     @Override
//     public void initialize() {
//         // Runs once when scheduled

//         s_Lights.Aim();
//     }

//     @Override
//     public void execute() {
//         // Runs repeatedly while scheduled
//     }

//     @Override
//     public void end(boolean interrupted) {
//         // Runs once when the command ends
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
