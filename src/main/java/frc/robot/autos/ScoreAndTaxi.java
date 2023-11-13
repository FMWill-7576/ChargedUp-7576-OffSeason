// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;




import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveForward;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.ScoreHighCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;



public class ScoreAndTaxi extends SequentialCommandGroup {
  
  public ScoreAndTaxi(Swerve s_Swerve, Gripper s_Gripper, Arm s_Arm) {

            

    addCommands(
      new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
     
     new ScoreHighCommand(s_Arm, s_Gripper),
     s_Arm.run(() -> s_Arm.armGrip()).withTimeout(1.2),
     new DriveForward(s_Swerve).raceWith(s_Gripper.run(() -> s_Gripper.hold())).withTimeout(2.65),
      // new InstantCommand( () -> s_Swerve.drive
      // (new Translation2d(0.35,0).times(Constants.Swerve.maxSpeed),
       // 0, true, true)).withTimeout(2.0),
        new InstantCommand( () -> s_Swerve.drive(new Translation2d( 0,0), 0, true, true), s_Swerve),
        new RotateCommand(s_Swerve).withTimeout(1.5)
    
     );
  
  }
}
