// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Arm() {}

  private final CANSparkMax armMotor = new CANSparkMax(13, MotorType.kBrushless);

  private final RelativeEncoder armEncoder = armMotor.getEncoder();
  private final PIDController armController = new PIDController(3, 0.0, 0.03);
  private final PIDController armController2 = new PIDController(1, 0, 0.06);

  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command armFloatingCommand() {
    return run(()->
    armMotor.set(armController.calculate(armEncoder.getPosition(), -0.6)))
    .withName("Float Arm");
  }
 
  public Command armToAmpCommand() {
    return run(() -> 
      armMotor.set(armController.calculate(armEncoder.getPosition(), -0.87)))
          .withName("Arm to Amp");
        }
    
  public Command armToSpeakerCommand() {
      return run(() -> 
        armMotor.set(armController.calculate(armEncoder.getPosition(), -0.89
         )))
          .withTimeout(2)
            .withName("Arm to Speaker");
        }

  public Command armToIntakeCommand1() {
        return run(() -> 
          armMotor.set(armController2.calculate(armEncoder.getPosition(), -0.12)))
              .withName("Arm to Intake 1");
        } 
  public Command StopArm() {
    return run(() -> armMotor.set(0)).withName("Stop Arm");
  }

  @Override
  public void periodic() {
    //armController2.setTolerance(0.001);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition());
    SmartDashboard.putNumber("PID Output IntakePOS", armController2.calculate(armEncoder.getPosition(), -0.1));
    SmartDashboard.putNumber("PID Output SpeakerPOS", armController.calculate(armEncoder.getPosition(), -0.8));
  }
}
