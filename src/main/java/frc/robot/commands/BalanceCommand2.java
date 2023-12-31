// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** Uses the gyroscope to balance the robot on the charger. */
public class BalanceCommand2 extends CommandBase {
  private final Swerve s_Swerve;

  /**
   * Creates a new {@link BalanceCommand2}.
   * 
   * @param subsystem The required subsystem.
   */
  public BalanceCommand2(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }
 double initRoll = DriveForward.initialRoll;
 double tolerance = 4.0;
  @Override
  public void execute() {

    if (s_Swerve.getRoll() > initRoll + tolerance){
      
      s_Swerve.drive(
      
    new Translation2d(-0.063, 
    0).times(Constants.Swerve.maxSpeed),
      0,
      true,
      true);

    }
    
    else if(s_Swerve.getRoll() < -(initRoll + tolerance)){

  s_Swerve.drive(
      
    new Translation2d(0.07, 
    0).times(Constants.Swerve.maxSpeed),
      0,
      true,
      true);

    }

    else{

       s_Swerve.drive(
        new Translation2d(0,  0),
        0,
        true,
        true); 
        s_Swerve.xLock();
    }

  
}
  @Override
  public void end(boolean interrupted) {
   // s_Swerve.drive(new Translation2d(0,0), 0.0, true, false);
    s_Swerve.xLock();
  }

  @Override
  public boolean isFinished() {
    //return (s_Swerve.getRoll() < (initRoll + tolerance)) && (s_Swerve.getRoll() > -(initRoll + tolerance));
    return false;
  }
}