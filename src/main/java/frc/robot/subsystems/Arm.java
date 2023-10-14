package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.SparkMaxPIDController;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
@SuppressWarnings("unused")

public class Arm extends SubsystemBase {
  private static double kS = 0.3;
  private static double kG = 0.65;
  private static double kV = 0.13;
  public double calculatedkG;
  private ArmFeedforward armFeedforward;
  public static double armSpeedRate =  1.0;
  private CANSparkMax armMotor;
  private CANSparkMax armMotor2;
  private final SparkMaxPIDController armController;
  private final SparkMaxPIDController armController2;
  private RelativeEncoder integratedArmEncoder;
  private RelativeEncoder integratedArmEncoder2;
  private final TrapezoidProfile.Constraints m_constraints;
  private  TrapezoidProfile.State  m_start;
  private TrapezoidProfile.State  m_end;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile.State m_setpoint;
  private static double kDt;

  public Arm() {
      armMotor  = new CANSparkMax(9, MotorType.kBrushless);
      armMotor2  = new CANSparkMax(10, MotorType.kBrushless);
      m_constraints = new TrapezoidProfile.Constraints(2, 0.3);
      armController = armMotor.getPIDController();
      armController2 = armMotor2.getPIDController();
      armFeedforward = new ArmFeedforward(kS, kG, kV);
     m_start = new TrapezoidProfile.State(-68, 0);
     m_end = new TrapezoidProfile.State(30, 0);
     profile = new TrapezoidProfile(m_constraints,m_end,m_start);
      m_goal = new TrapezoidProfile.State();
      m_setpoint = new TrapezoidProfile.State();
       kDt = 0.02;
  
     integratedArmEncoder = armMotor.getEncoder();
     integratedArmEncoder2 = armMotor2.getEncoder();
     armMotorConfig();

  }
 



      public void armMotorConfig() { 
        armMotor.restoreFactoryDefaults();
        armMotor2.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kPositionOnly);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor2, Usage.kPositionOnly);
        armMotor.setSmartCurrentLimit(40);
        armMotor2.setSmartCurrentLimit(40);
        armMotor.setInverted(true);
        armMotor.setIdleMode(Constants.ArmConstants.armNeutralMode);
        armMotor2.setInverted(true);
        armMotor2.setIdleMode(Constants.ArmConstants.armNeutralMode);
        //armMotor2.follow(armMotor);
         
        integratedArmEncoder.setPositionConversionFactor(9.29); 
        integratedArmEncoder.setPosition(-68.0);
       
        integratedArmEncoder2.setPositionConversionFactor(9.29); 
        integratedArmEncoder2.setPosition(-68.0); 
        armController.setP(Constants.ArmConstants.armKP);
        armController.setI(Constants.ArmConstants.armKI);
        armController.setD(Constants.ArmConstants.armKD);
        armController2.setP(Constants.ArmConstants.armKP);
        armController2.setI(Constants.ArmConstants.armKI);
        armController2.setD(Constants.ArmConstants.armKD);
        //armController.setFF(Constants.ArmConstants.armKFF);
        armMotor.enableVoltageCompensation(Constants.ArmConstants.voltageComp);
        armMotor2.enableVoltageCompensation(12.0);
        //integratedArmEncoder.setReverseDirection(false); // bu
        //integratedArmEncoder.setDistancePerPulse(Constants.ArmConstants.armConversionPositionFactor); //bu
        armMotor.burnFlash();
       //armMotor2.follow(armMotor);
        armMotor2.burnFlash();
    }

      public void armSet(Rotation2d angle) {
        m_goal = new TrapezoidProfile.State(angle.getDegrees(), 0);


        armController.setReference(
        m_setpoint.position,
        CANSparkMax.ControlType.kPosition,
        0,
        0.3,
        ArbFFUnits.kVoltage);

        armController2.setReference(
          m_setpoint.position,
          ControlType.kPosition,
          0,
          0.3,
          ArbFFUnits.kVoltage);
          
      
      }
     

     public void armUp(){
     // armSet(Rotation2d.fromDegrees(150.0));
     armDrive(0.205);
     }

     public void armCone(){
       armSet(Rotation2d.fromDegrees(30.0));
      }

       public void armCube(){
        armSet(Rotation2d.fromDegrees(140.0));}

     public void armDrive(double armPercentage){
  armMotor.set(armPercentage);
  armMotor2.set(armPercentage);
      }    

     public void armHome(){
      armSet(Rotation2d.fromDegrees(-68.0));
     }

     public void armDown(){
      //armSet(Rotation2d.fromDegrees(200.0));
      armDrive(-0.205);
     }


     public void armReset(){
      integratedArmEncoder.setPosition(-68.0);
      integratedArmEncoder2.setPosition(-68.0);
     }

    @Override
    public void periodic() {
      //if (integratedArmEncoder.getPosition() > 85.0 && integratedArmEncoder.getPosition() < 95.0 )
     // {calculatedkG = 0.0; }
     // else if (integratedArmEncoder.getPosition()> 95.0){
      //  calculatedkG = -0.07;
     // }
     // else {
     // calculatedkG = 0.06 ; }
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("arm encoder" , integratedArmEncoder.getPosition());
      m_setpoint = profile.calculate(kDt);
      var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

      //SmartDashboard.putNumber("arm distance" , integratedArmEncoder.getDistance());
    }
}