// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveBack;
import frc.robot.commands.DriveForward;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.ScoreHighCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;



public class ScoreTaxiAndBalance extends SequentialCommandGroup {
  
  public ScoreTaxiAndBalance(Swerve s_Swerve, Gripper s_Gripper, Arm s_Arm) {
  

    addCommands(
      
      new ScoreHighCommand(s_Arm, s_Gripper),
      s_Arm.run(() -> s_Arm.armHome()).withTimeout(0.85),
      new DriveForward(s_Swerve),
      new WaitCommand(2.3), 
      new RotateCommand(s_Swerve).withTimeout(1.0),
      new DriveBack(s_Swerve),
      new WaitCommand(1.0),
      
       new BalanceCommand(s_Swerve) 
       
   
 
     );
  
  }
}
