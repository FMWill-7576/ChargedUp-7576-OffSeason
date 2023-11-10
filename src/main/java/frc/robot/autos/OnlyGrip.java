package frc.robot.autos;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ScoreHighCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;

public class OnlyGrip extends SequentialCommandGroup {
  public OnlyGrip(Swerve s_Swerve, Gripper s_Gripper, Arm s_Arm ) {

    // An example trajectory to follow.  All units in meters.

    addCommands(
      new ScoreHighCommand(s_Arm, s_Gripper),
      s_Arm.run(() -> s_Arm.armHome()).withTimeout(0.85),
      new InstantCommand(() -> s_Swerve.drive(
            new Translation2d(0, 0),
            0,
            false,
            true))     
     );     
  }
}