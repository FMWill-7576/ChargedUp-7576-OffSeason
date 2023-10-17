// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */


public class RobotContainer {
  /* Controllers */
   private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);
  

  /* Drive Controls */
  //private final int translationAxis = XboxController.Axis.kLeftY.value;
  //private final int strafeAxis = XboxController.Axis.kLeftX.value;
  //private final int rotationAxis = XboxController.Axis.kRightX.value; 

 private final int translationAxis = 1;
 private final int strafeAxis = 0;
 private final int rotationAxis = 4; 
 private final int armAxis = 1;
 private final int armUpAxis = 3; 
 private final int armDownAxis = 2;

 
 
  /* Driver Buttons */

  // private final JoystickButton zeroGyro =
  // new JoystickButton(driver, XboxController.Button.kY.value);
      private final JoystickButton robotCentric =
      new JoystickButton(driver, 6);    

      private final JoystickButton zeroGyro =
      new JoystickButton(driver, 5);

      private final JoystickButton incSpeed =
      new JoystickButton(driver, 5);

      private final JoystickButton decSpeed =
      new JoystickButton(driver, 3);

      private final JoystickButton xLock = 
      new JoystickButton(driver, 3);
      private final JoystickButton resetAbsolute2 = 
      new JoystickButton(driver, 8);
 

      private final JoystickButton armReset = 
      new JoystickButton(operator,7);
      private final Trigger armDown = 
      new Trigger(() -> operator.getRawAxis(armDownAxis) > 0.3);
      private final Trigger armUp = 
     new Trigger (() -> operator.getRawAxis(armUpAxis) > 0.3); 
     private final JoystickButton armCone = 
     new JoystickButton(operator,2);
     private final JoystickButton armHome = 
      new JoystickButton(operator,1);

    

     // private final JoystickButton pistonTest = 
     // new JoystickButton(driver, 6);


      private final JoystickButton resetAbsolute =
      new JoystickButton(driver,1);

      private final JoystickButton intake =
      new JoystickButton(operator,5);

      private final JoystickButton outake =
      new JoystickButton(operator,4);

      private final JoystickButton hold = 
      new JoystickButton(operator,3);

      private final JoystickButton drop =
      new JoystickButton(operator,6);

      private final JoystickButton armGrip = 
      new JoystickButton(operator, 8); 

      


  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Gripper s_Gripper = new Gripper();
   private final Arm s_Arm = new Arm();
   //private final Vision s_Vision = new Vision();

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    s_Swerve::getPose, // Pose2d supplier
    s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
);
// This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
/*  for every path in the group
 List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));
*/

   
   // private final Command exampleAuto = new exampleAuto(s_Swerve, s_Gripper);
    private final Command ScoreTaxiAndBalance = new ScoreTaxiAndBalance(s_Swerve, s_Gripper, s_Arm);
   // private final Command driveStraight = new driveStraight(s_Swerve, s_Gripper);
    private final Command doNothing = new doNothing(s_Swerve);
    private final Command GripOnly = new OnlyGrip(s_Swerve, s_Gripper);
    private final Command ScoreAndBalance = new ScoreAndBalance(s_Swerve, s_Gripper, s_Arm);
    private final Command ScoreAndTaxi = new ScoreAndTaxi(s_Swerve, s_Gripper, s_Arm);
    // Command fullAuto = autoBuilder.fullAuto(pathGroup);
    
    // A chooser for autonomous commands
     SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("place high", new InstantCommand(() -> s_Arm.armCone()));

        s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> Math.pow(-driver.getRawAxis(translationAxis) * 1, 1),
            () -> Math.pow(-driver.getRawAxis(strafeAxis) * 1 ,1),
            () -> Math.pow(driver.getRawAxis(rotationAxis) * 0.45 ,1),
            () -> robotCentric.getAsBoolean())); 



         s_Gripper.setDefaultCommand(
          s_Gripper.run(() -> s_Gripper.stop()));

          s_Arm.setDefaultCommand( 
          new ArmCommand(
            s_Arm,
           () -> (-operator.getRawAxis(armAxis) * 0))  
          //s_Arm.run(() -> s_Arm.armDrive(1.0))


         );   
            // Add commands to the autonomous command chooser
      m_chooser.setDefaultOption("score+taxi+denge", ScoreTaxiAndBalance);
      //m_chooser.addOption("FULL RUTIN", exampleAuto);
      m_chooser.addOption("score-denge", ScoreAndBalance);
      m_chooser.addOption("nothing", doNothing);
      m_chooser.addOption("score-taxi", ScoreAndTaxi);
      m_chooser.addOption("sadece gripper", GripOnly);
        // Put the chooser on the dashboard
        SmartDashboard.putData("OTONOM", m_chooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    incSpeed.whileTrue(new InstantCommand(() -> s_Swerve.incSpeed()));
    decSpeed.whileTrue(new InstantCommand(() -> s_Swerve.decSpeed()));
    xLock.whileTrue(s_Swerve.run(() -> s_Swerve.xLock()));
    //armTesting.whileTrue(s_Arm.run(() -> s_Arm.armTesting()));
    resetAbsolute.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
    resetAbsolute2.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
   // armUp.whileTrue(s_VictorArm.run(() -> s_VictorArm.armUp()));
   // armDown.whileTrue(s_VictorArm.run(() -> s_VictorArm.armDown()));
    //pistonTest.onTrue(s_Gripper.run(() -> s_Gripper.pistonTest()));
   // armUp.whileTrue(s_Slider.run(() -> s_Slider.armUp()));
   // armDown.whileTrue(s_Slider.run(() -> s_Slider.armDown()));
    armUp.whileTrue(s_Arm.run(() -> s_Arm.armUp()));
    armDown.whileTrue(s_Arm.run(() -> s_Arm.armDown()));
    armHome.toggleOnTrue(s_Arm.run(() -> s_Arm.armHome()));
    armReset.whileTrue(s_Arm.runOnce(() -> s_Arm.armReset()));
    armCone.toggleOnTrue(s_Arm.run(() -> s_Arm.armCone()));
    armGrip.toggleOnTrue(s_Arm.run(() -> s_Arm.armGrip()).alongWith(s_Gripper.run(() -> s_Gripper.hold())));
    intake.whileTrue(s_Gripper.run(() -> s_Gripper.intake()));
    outake.whileTrue(s_Gripper.run(() -> s_Gripper.outake()));
    drop.whileTrue(s_Gripper.run(() -> s_Gripper.drop()));
    hold.toggleOnTrue(s_Gripper.run(() -> s_Gripper.hold()));
  //  resetSlider.onTrue(new InstantCommand(() -> s_Slider.resetSlider()));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new exampleAuto(s_Swerve);
    return m_chooser.getSelected();
  }
}
