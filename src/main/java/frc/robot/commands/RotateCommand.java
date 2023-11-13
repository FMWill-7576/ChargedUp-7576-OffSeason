// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class RotateCommand extends CommandBase {
  static double initialRoll;
  private final Swerve s_Swerve;
  PIDController headingController;
  /** Creates a new RotateCommand. */
  public RotateCommand(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    headingController = new PIDController(0.014, 0, 0);
    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.drive(
      
      new Translation2d(0, 
      0), 

        MathUtil.clamp(
          headingController.calculate(
            Math.abs(s_Swerve.getYaw().getDegrees()),
             180),
             -0.35,
             0.35)*Constants.Swerve.maxAngularVelocity,

        true,
        true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new InstantCommand(() -> s_Swerve.drive(
      new Translation2d(0, 0),
      0,
      true,
      true));  
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(s_Swerve.getYaw().getDegrees())>161.0) ;
  }
}
