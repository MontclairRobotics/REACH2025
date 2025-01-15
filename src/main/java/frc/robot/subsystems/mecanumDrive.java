// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class MecanumDrive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public MecanumDrive() {}
  private Translation2d frontLeftLocation = new Translation2d(1,2);
  private Translation2d backLeftLocation = new Translation2d(3,4);
  private Translation2d fontRightLocation = new Translation2d(5,6);
  private Translation2d backRightLocation = new Translation2d(7,8);

  private SparkMax frontLeftMotor =new SparkMax(0, MotorType.kBrushless);
  private SparkMax backLeftMotor =new SparkMax(0, MotorType.kBrushless);
  private SparkMax frontRightMotor =new SparkMax(0, MotorType.kBrushless);
  private SparkMax backRightMotor =new SparkMax(0, MotorType.kBrushless);
  MecanumDriveKinematics driveKinematics = new MecanumDriveKinematics(frontLeftLocation,fontRightLocation,backLeftLocation,backRightLocation);
  double MaxSpeed = 2;

  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
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
  public void drive(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(MaxSpeed);

    frontLeftMotor.set(wheelSpeeds.frontLeftMetersPerSecond / MaxSpeed);
    backLeftMotor.set(wheelSpeeds.rearLeftMetersPerSecond / MaxSpeed);
    frontRightMotor.set(wheelSpeeds.frontRightMetersPerSecond / MaxSpeed);
    backRightMotor.set(wheelSpeeds.rearRightMetersPerSecond / MaxSpeed);
  }
  public void driveFromController(CommandPS5Controller controller) {
    // Query some boolean state, such as a digital sensor.
    double xSpeed = MaxSpeed*(controller.getLeftX());
    double ySpeed = MaxSpeed*(-controller.getLeftY());
    double turnSpeed = MaxSpeed*(controller.getRightX());
    drive(new ChassisSpeeds(xSpeed,ySpeed,turnSpeed));

  }

  public Command driveDefault(CommandPS5Controller controller){
    return Commands.run(() -> driveFromController(controller));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
