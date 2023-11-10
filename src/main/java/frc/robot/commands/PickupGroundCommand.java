package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

public class PickupGroundCommand extends SequentialCommandGroup {
  public PickupGroundCommand(Arm s_Arm, Gripper s_Gripper ) {


    addCommands(
      s_Arm.run(() -> s_Arm.armGrip()).withTimeout(1.5),

       
       s_Gripper.run(() -> s_Gripper.hold()).withTimeout(0.8)  
     );     
  }
}