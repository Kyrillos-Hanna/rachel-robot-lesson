// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants.DrivebaseConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.GearRatioConstants;

public class Drivebase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 
  public static Drivebase m_instance = new Drivebase();

  CANSparkMax m_leftSlave = new CANSparkMax(DrivebaseConstants.kLeftSlave, MotorType.kBrushless);  //brushless > brush
  CANSparkMax m_leftMaster = new CANSparkMax(DrivebaseConstants.kLeftMaster, MotorType.kBrushless);
  CANSparkMax m_rightSlave = new CANSparkMax(DrivebaseConstants.kRightSlave, MotorType.kBrushless);
  CANSparkMax m_rightMaster = new CANSparkMax(DrivebaseConstants.kRightMaster, MotorType.kBrushless);

  RelativeEncoder m_leftEncoder  = m_leftMaster.getAlternateEncoder(DrivebaseConstants.kCountsPerRev);  //creates an encoder 
  RelativeEncoder m_rightEncoder = m_rightMaster.getAlternateEncoder(DrivebaseConstants.kCountsPerRev); 

  DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftMaster,m_rightMaster); //MAKE NEW DRIVEBASE

   public Drivebase() {
    configMotors();  //
  }


  private void configMotors(){  
    m_leftSlave.follow(m_leftMaster); //slave motor 
    m_rightSlave.follow(m_rightMaster);

    m_leftMaster.setInverted(true); //invert motor 
    m_leftSlave.setInverted(m_leftMaster.getInverted());

    m_leftMaster.restoreFactoryDefaults();   //restores factor defeault defeault
    m_leftSlave.restoreFactoryDefaults();
    m_rightMaster.restoreFactoryDefaults();   
    m_rightSlave.restoreFactoryDefaults();
    

    m_leftMaster.setIdleMode(IdleMode.kBrake);              //sets break or coast
    m_leftSlave.setIdleMode(m_leftMaster.getIdleMode());
    m_rightMaster.setIdleMode(m_leftMaster.getIdleMode());
    m_rightSlave.setIdleMode(m_leftMaster.getIdleMode());

    m_leftMaster.setSmartCurrentLimit(DrivebaseConstants.kStallLimit,DrivebaseConstants.kFreeLimit); //sets stall and free limits 
    m_leftSlave.setSmartCurrentLimit(DrivebaseConstants.kStallLimit,DrivebaseConstants.kFreeLimit);
    m_rightMaster.setSmartCurrentLimit(DrivebaseConstants.kStallLimit,DrivebaseConstants.kFreeLimit);
    m_rightSlave.setSmartCurrentLimit(DrivebaseConstants.kStallLimit,DrivebaseConstants.kFreeLimit);


    //stallimit - minimum amount of torque that the motor needs to "break the stall" 
    //free - maximum amount of power distributed into the moto
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  public CommandBase die() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   * 
   * 
   *
   */

   public void limitedArcadeDrive(double speed, double rotation){   //runs arcade drive
      m_differentialDrive.arcadeDrive(speed*0.5, rotation*0.5); //half power so motors don't burn out
   }

    public void arcadeDrive(double speed, double rotation){
      m_differentialDrive.arcadeDrive(speed, rotation); //full power when u need more power
   }

   public double getAverageDistance(){ // the left distance + right distance divided by two to get the average distance of both encoders
      return (getLeftDistance() + getRightDistance())/2;
   }

  public double getLeftDistance(){ 
      double circumference = Math.PI * DrivebaseConstants.kWheelDiameterInches; //stores the circumference variable; circumference = 
      double revolutions = (m_leftEncoder.getPosition() / m_leftEncoder.getCountsPerRevolution());
      //distance = circumference * revolutions  (ARC LENGTH)
      return (circumference * revolutions)/GearRatioConstants.kGearRatio;
  }

  public double getRightDistance(){
    double circumference = Math.PI * DrivebaseConstants.kWheelDiameterInches; //FINDS CIRCUMFRENCE
    double revolutions = m_rightEncoder.getPosition() / m_rightEncoder.getCountsPerRevolution(); //FINDS REVOLUTION
    return (circumference * revolutions); //GETS DISTANCE BY MULTIPLYING CIRCUMFRENCE AND REVOLUTIONS
  }

  public void resetEncoders(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }
  
  public void LogData(){
    SmartDashboard.putNumber("left distance", getLeftDistance()); //DISPLAYS LEFT DISTANCE
    SmartDashboard.putNumber("right distance", getRightDistance()); //DISPLAYS RIGHT DISTANCE
    SmartDashboard.putNumber("average distance", getAverageDistance()); //DISPLAYS AVERAGE DISTANCE
  }



  
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
      LogData();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
