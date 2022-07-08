// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {
  private final WPI_TalonFX frontLeft = MotorHelper.createFalconMotor(kDrivetrain.FRONT_LEFT_ID,
      kDrivetrain.CURRENT_LIMIT, NeutralMode.Coast, TalonFXInvertType.CounterClockwise);
  private final WPI_TalonFX frontRight = MotorHelper.createFalconMotor(kDrivetrain.FRONT_RIGHT_ID,
      kDrivetrain.CURRENT_LIMIT, NeutralMode.Coast, TalonFXInvertType.Clockwise);
  private final WPI_TalonFX backLeft = MotorHelper.createFalconMotor(kDrivetrain.BACK_LEFT_ID,
      kDrivetrain.CURRENT_LIMIT, NeutralMode.Coast, TalonFXInvertType.CounterClockwise);
  private final WPI_TalonFX backRight = MotorHelper.createFalconMotor(kDrivetrain.BACK_RIGHT_ID,
      kDrivetrain.CURRENT_LIMIT, NeutralMode.Coast, TalonFXInvertType.Clockwise);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft, frontRight);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final Field2d field = new Field2d();

  private float zeroOffset = 0;
  private boolean calibrating = true;
  private DifferentialDriveOdometry odomotry = new DifferentialDriveOdometry(new Rotation2d(0));

  public Drivetrain() {
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    SmartDashboard.putData("Field", field);
  }
  
  public void curveDrive( double linearVelocity, double angularVelocity, boolean isQuickTurn ) {
    differentialDrive.curvatureDrive(linearVelocity, angularVelocity, isQuickTurn);
  }

  @Override
  public void periodic() {
    if (!gyro.isCalibrating() && calibrating) {
      zeroOffset = gyro.getYaw();
      calibrating = false;
    }

    odomotry.update(
      new Rotation2d(Math.toRadians(Conversion.normalizeGyro(gyro.getYaw() - zeroOffset) * kDrivetrain.GYRO_INVERSION)), 
      Conversion.ticksToMeters(frontLeft.getSelectedSensorPosition(), kDrivetrain.WHEEL_DIAMTER), 
      Conversion.ticksToMeters(frontRight.getSelectedSensorPosition(), kDrivetrain.WHEEL_DIAMTER)
    );

    field.setRobotPose(odomotry.getPoseMeters());
    SmartDashboard.putNumber("heading ", Conversion.normalizeGyro(gyro.getYaw() - zeroOffset) * kDrivetrain.GYRO_INVERSION);
    SmartDashboard.putNumber("velocity", Conversion.ticksToMeters(frontLeft.getSelectedSensorVelocity(), kDrivetrain.WHEEL_DIAMTER));
    SmartDashboard.putNumber("odometry x", odomotry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometry y", odomotry.getPoseMeters().getY());
  }

  @Override
  public void simulationPeriodic() { }

  public Pose2d getPose() {
    return odomotry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(Conversion.ticksToMeters(frontLeft.getSelectedSensorPosition(), kDrivetrain.WHEEL_DIAMTER), Conversion.ticksToMeters(frontLeft.getSelectedSensorPosition(), kDrivetrain.WHEEL_DIAMTER));
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odomotry.resetPosition(pose, gyro.getRotation2d());
  }

  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public void resetEncoders() {
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (Conversion.ticksToMeters(frontLeft.getSelectedSensorPosition(), kDrivetrain.WHEEL_DIAMTER) + Conversion.ticksToMeters(frontRight.getSelectedSensorPosition(), kDrivetrain.WHEEL_DIAMTER)) / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.getYaw();
  }


  public double getHeading() {
    return Conversion.normalizeGyro((gyro.getYaw() - zeroOffset) * kDrivetrain.GYRO_INVERSION);
  }

  public double getTurnRate() {
    return gyro.getRate();
  }
}
