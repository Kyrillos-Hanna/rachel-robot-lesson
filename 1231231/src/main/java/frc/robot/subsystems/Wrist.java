// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class Wrist extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public enum WristState {
    OFF,
    JOG,
    POSITION,
    ZERO
  }

  CANSparkMax m_leftMotor = new CANSparkMax(WristConstants.kLeft, MotorType.kBrushless);
  CANSparkMax m_rightMotor = new CANSparkMax(WristConstants.kRight, MotorType.kBrushless);

  public DigitalInput m_armLimitSwitch = new DigitalInput(WristConstants.kChannel);

  SparkMaxPIDController m_PIDController = m_leftMotor.getPIDController();

  WristState state = WristState.OFF;

  double jogValue = 0;
  Rotation2d setpoint = new Rotation2d();

  public Wrist() {
    configMotors();
  }

  public void ZER0() {
    if(m_armLimitSwitch.get()){
      jogValue = 0;
    } else {
      jogValue = 0.1;
    }
  }
  public double getJog() {
    return this.jogValue;
  }

  public WristState getState() {
    return this.state;
  }

  public Rotation2d getSetpoint() {
    return this.setpoint;
  }

  public void setJog(double jogValue) {
    this.jogValue = jogValue;
  }

  public void setState(WristState state) {
    this.state = state;
  }

  public void setSetpoint(Rotation2d setpoint) {
    this.setpoint = setpoint;
  }

  public void OFF() {
    m_rightMotor.set(0);
    m_leftMotor.set(0);
  }

  public void goToPosition() {
    m_PIDController.setReference(setpoint.getRotations() * WristConstants.kGearRatio, ControlType.kSmartMotion);
  }

  public void set(double value) {
    m_leftMotor.set(value);
    
  }
  

  SparkMaxAbsoluteEncoder m_leftEncoder = m_leftMotor.getAbsoluteEncoder(Type.kDutyCycle);

  static Wrist m_instance = new Wrist();

  private static Wrist getInstance() {
    return m_instance;
  }

  private void configMotors() {
    
    m_rightMotor.follow(m_leftMotor);

    m_rightMotor.setInverted(true);

    m_rightMotor.restoreFactoryDefaults();   
    m_leftMotor.restoreFactoryDefaults();

    m_leftMotor.setIdleMode(IdleMode.kBrake);           
    m_rightMotor.setIdleMode(m_leftMotor.getIdleMode());

    m_rightMotor.setSmartCurrentLimit(WristConstants.kStallLimit,WristConstants.kFreeLimit);
    m_leftMotor.setSmartCurrentLimit(WristConstants.kStallLimit,WristConstants.kFreeLimit);

    m_PIDController.setP(WristConstants.Kp);
    m_PIDController.setP(WristConstants.Ki);
    m_PIDController.setP(WristConstants.Kd);
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

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    switch(state) {
      case OFF:
        OFF();
        break;
      case JOG:
        set(jogValue);
        break;
      case POSITION:
        //idk rn
      case ZERO:
        ZER0();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
