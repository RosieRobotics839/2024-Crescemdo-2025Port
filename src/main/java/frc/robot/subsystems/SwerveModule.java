// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.kDriveTrain.DriveConstants;
import frc.robot.Constants.kDriveTrain.kSwerveModule;
import frc.utils.FirstOrderLag;
import frc.utils.CANSparkMax.MyCANSparkMax;
import frc.robot.Robot;
import frc.robot.Constants.CANID_t;
public class SwerveModule extends SubsystemBase {

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Drivetrain/wheel");

  public MyCANSparkMax m_motorDrive, m_motorSteer;
  public RelativeEncoder m_encoderDrive, m_encoderSteer;
  public SparkClosedLoopController m_pidDrive, m_pidSteer;
  int m_steeringOffset;
  SwerveModuleState optimizedState = new SwerveModuleState(0, new Rotation2d(0));

  public boolean m_setupDriveDone = false;
  public boolean m_setupSteerDone = false;

  public AnalogInput m_analogEncoder;
  public double angleCalibration;
  public FirstOrderLag m_magLimiter = new FirstOrderLag(DriveConstants.kLinearAccelerationTau, 0, 0.020);

  DoublePublisher 
    nt_angleinit,
    nt_angle,
    nt_speedcmd,
    nt_anglecmd,
    nt_speed, nt_i, nt_analog;

  public SwerveModule(CANID_t CANID, double angleCalibration, String name) {
   
    // Setup Network Table Publishers
    nt_angleinit = table.getDoubleTopic("angle/init/"+name).publish();
    nt_angle = table.getDoubleTopic("angle/"+name).publish();
    nt_speed = table.getDoubleTopic("speed/"+name).publish();
    nt_speedcmd = table.getDoubleTopic("speedcmd/"+name).publish();
    nt_anglecmd = table.getDoubleTopic("anglecmd/"+name).publish();
    nt_i = table.getDoubleTopic("current/"+name).publish();
    nt_analog = table.getDoubleTopic("analog/"+name).publish();

    /* Define drive motor controller. */
    m_motorDrive = new MyCANSparkMax(CANID.driving, MotorType.kBrushless);

    /* Define steer motor controller. */
    m_motorSteer = new MyCANSparkMax(CANID.steering, MotorType.kBrushless);

    /* Define steer analog encoder and store calibration value */
    m_analogEncoder = new AnalogInput(CANID.encoder);
    this.angleCalibration = angleCalibration;
    
    m_setupDriving.ignoringDisable(true).schedule();
    m_setupSteering.ignoringDisable(true).schedule();

    setState(optimizedState);
  }

  Command m_setupDriving = Commands.sequence(
    Commands.waitUntil(() -> m_motorDrive.restoreFactoryDefaults() == REVLibError.kOk),
    Commands.waitUntil(() -> (m_encoderDrive = m_motorDrive.getEncoder()) != null),
    Commands.waitUntil(() -> m_encoderDrive.setPositionConversionFactor((Robot.isSimulation() ? 60: kSwerveModule.kDriveEncoderPositionFactor)) == REVLibError.kOk),
    Commands.waitUntil(() -> m_encoderDrive.setVelocityConversionFactor(kSwerveModule.kDriveEncoderVelocityFactor) == REVLibError.kOk),
    Commands.waitUntil(() -> (m_pidDrive = m_motorDrive.getPIDController()) != null),
    Commands.waitUntil(() -> m_pidDrive.setFeedbackDevice(m_encoderDrive) == REVLibError.kOk),
    Commands.waitUntil(() -> m_motorDrive.setSmartCurrentLimit((int)kSwerveModule.kDrivingMotorCurrentLimit) == REVLibError.kOk),
    Commands.waitUntil(() -> m_motorDrive.setIdleMode(IdleMode.kBrake) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidDrive.setP(kSwerveModule.kDriveKp) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidDrive.setI(kSwerveModule.kDriveKi) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidDrive.setD(kSwerveModule.kDriveKd) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidDrive.setFF(kSwerveModule.kDriveKff) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidDrive.setOutputRange(-1,1) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidDrive.setIZone(0.15) == REVLibError.kOk),
    Commands.waitUntil(() -> m_motorDrive.setInverted2(false) == REVLibError.kOk),
    Commands.waitUntil(() -> m_motorDrive.burnFlash() == REVLibError.kOk),
    new InstantCommand(()-> m_setupDriveDone = true)
  );

  Command m_setupSteering = Commands.sequence(
    Commands.waitUntil(() -> m_motorSteer.restoreFactoryDefaults() == REVLibError.kOk),
    Commands.waitUntil(() -> {m_steeringOffset = m_analogEncoder.getValue(); nt_angleinit.set(m_steeringOffset); return true;}),
    Commands.waitUntil(() -> (m_encoderSteer = m_motorSteer.getEncoder()) != null),
    Commands.waitUntil(() -> m_encoderSteer.setPositionConversionFactor(kSwerveModule.kSteerEncoderPositionFactor) == REVLibError.kOk),
    Commands.waitUntil(() -> m_encoderSteer.setPosition(-(m_steeringOffset-angleCalibration)/4096.0 * (2*Math.PI)) == REVLibError.kOk),
    Commands.waitUntil(() -> (m_pidSteer = m_motorSteer.getPIDController()) != null),
    Commands.waitUntil(() -> m_pidSteer.setFeedbackDevice(m_encoderSteer) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidSteer.setPositionPIDWrappingEnabled(true) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidSteer.setPositionPIDWrappingMinInput(-Math.PI) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidSteer.setPositionPIDWrappingMaxInput(Math.PI) == REVLibError.kOk),
    Commands.waitUntil(() -> m_motorSteer.setSmartCurrentLimit((int)kSwerveModule.kSteeringMotorCurrentLimit) == REVLibError.kOk),
    Commands.waitUntil(() -> m_motorSteer.setIdleMode(IdleMode.kBrake) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidSteer.setP(kSwerveModule.kSteerKp) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidSteer.setI(kSwerveModule.kSteerKi) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidSteer.setD(kSwerveModule.kSteerKd) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidSteer.setFF(kSwerveModule.kSteerKff) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidSteer.setOutputRange(-1,1) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidSteer.setIZone(0.05) == REVLibError.kOk),
    Commands.waitUntil(() -> m_motorSteer.setInverted2(true) == REVLibError.kOk),
    Commands.waitUntil(() -> m_motorSteer.burnFlash() == REVLibError.kOk),
    new InstantCommand(()-> m_setupSteerDone = true)
  );

  public SwerveModuleState getState() {
    // Returns the velocity of the swerve drive wheel and angle
    return new SwerveModuleState(getDriveVelocity(),
        new Rotation2d(getSteerPosition()));
  }
  
  public SwerveModulePosition getPosition() {
    // Returns the position of the swerve module wheel and angle
    double angle;
    angle = getSteerPosition();
    return new SwerveModulePosition(getDrivePosition(),
        new Rotation2d(angle));
  }

  private double getDriveVelocity(){
    if (!m_setupDriveDone) return 0;
    return m_encoderDrive.getVelocity();
  }

  private double getDrivePosition(){
    if (!m_setupDriveDone) return 0;
    return m_encoderDrive.getPosition();
  }

  private double getSteerPosition(){
    if (!m_setupSteerDone) return 0;
    return m_encoderSteer.getPosition();
  }

  public void setSpeed(double speedMetersPerSecond){
    if (!m_setupDriveDone) return;
    optimizedState.speedMetersPerSecond = speedMetersPerSecond;
    m_pidDrive.setReference(speedMetersPerSecond,SparkMax.ControlType.kVelocity);
  }

  public void setState(SwerveModuleState targetState) {
    if (!m_setupDriveDone || !m_setupSteerDone) return;
    // Sets the target state of the swerve drive equal to the input state
    optimizedState = SwerveModuleState.optimize(targetState, new Rotation2d(getSteerPosition()));
    //m_pidSteer.setReference(0,CANSparkMax.ControlType.kPosition);
  }
  
  @Override
  public void periodic() {

    double speedcmd = m_magLimiter.calculate(optimizedState.speedMetersPerSecond);
    double anglecmd = optimizedState.angle.getRadians();
    
    if (m_setupDriveDone && m_setupSteerDone){
      m_pidDrive.setReference(speedcmd,SparkMax.ControlType.kVelocity);
      m_pidSteer.setReference(anglecmd,SparkMax.ControlType.kPosition);

      // This method will be called once per scheduler run
      nt_angle.set(getSteerPosition());
      nt_speed.set(getDriveVelocity());
      nt_anglecmd.set(anglecmd);
      nt_speedcmd.set(speedcmd);
      nt_i.set(m_motorDrive.getOutputCurrent());
    }
    nt_analog.set(m_analogEncoder.getValue());
  }
  
  static REVPhysicsSim sim = REVPhysicsSim.getInstance();
  public void simulationInit(){
    sim.addSparkMax(m_motorDrive, (float)2.6, (float)5676.0);
    sim.addSparkMax(m_motorSteer, (float)2.6, (float)5676.0);
  }

  public void simulationPeriodic(){
    if (!m_setupDriveDone || !m_setupSteerDone) return;
  
    if (!RobotController.isSysActive()){
      m_pidDrive.setReference(0,SparkMax.ControlType.kVelocity);
    } else {
      m_encoderSteer.setPosition(optimizedState.angle.getRadians());
    }
    sim.run();
  }
}
