// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.BalanceCommand2;
import frc.robot.commands.DriveForward2;
import frc.robot.commands.ScoreHighCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;


public class ScoreAndBalance extends SequentialCommandGroup {
  
  public ScoreAndBalance(Swerve s_Swerve, Gripper s_Gripper, Arm s_Arm) {

          

    addCommands(
      new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
   
       new ScoreHighCommand(s_Arm, s_Gripper),
       s_Arm.run(() -> s_Arm.armHome()).withTimeout(0.85),
       new DriveForward2(s_Swerve),
       new WaitCommand(1.15),
       new BalanceCommand(s_Swerve)
       
   
     );
  
  }
}
