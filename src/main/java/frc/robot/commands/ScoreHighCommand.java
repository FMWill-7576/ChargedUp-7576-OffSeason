package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

public class ScoreHighCommand extends SequentialCommandGroup {
  public ScoreHighCommand(Arm s_Arm, Gripper s_Gripper ) {


    addCommands(
      s_Arm.run(() -> s_Arm.armScore()).raceWith(
      s_Gripper.run(() -> s_Gripper.strongHold()))
     .withTimeout(0.85),

       s_Gripper.run(() -> s_Gripper.outake()).withTimeout(0.8),
       new InstantCommand(() -> s_Gripper.stop()) 
     );     
  }
}