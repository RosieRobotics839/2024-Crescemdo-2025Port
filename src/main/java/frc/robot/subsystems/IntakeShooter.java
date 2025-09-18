// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.CANSparkMax.MyCANSparkMax;
import frc.utils.CalibrationMap;
import frc.utils.VectorUtils;
import frc.utils.NTValues.NTBoolean;
import frc.utils.NTValues.NTDouble;

public class IntakeShooter extends SubsystemBase {
  private static IntakeShooter instance = new IntakeShooter();
  public static IntakeShooter getInstance() {
    return instance;
  }
  // Intake Variables
  NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("roboRIO/Intake");
  NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("roboRIO/Shooter");
  NetworkTable angleTable = NetworkTableInstance.getDefault().getTable("roboRIO/Angle");
  NetworkTable testtable = NetworkTableInstance.getDefault().getTable("roboRIO/CAUTION/TestInput");

  CalibrationMap m_angleCalibrationMap = new CalibrationMap(ShooterConstants.kShooterAngleCalibrationX, ShooterConstants.kShooterAngleCalibrationY);

  boolean m_usePositionTarget = false;
  double m_positionTarget = 0;
  double m_intakeTargetSpeed = 0; // Between 0 and 1, 0 = Stopped, 1 = Max speed

  public SparkClosedLoopController m_pidIntake;
  public SparkBaseConfig m_motorconfig;
  public MyCANSparkMax m_intake = new MyCANSparkMax(IntakeConstants.kCANID_Intake, MotorType.kBrushless);
  { if (Robot.isSimulation()){ REVPhysicsSim.getInstance().addSparkMax(m_intake, (float)2.6, (float)5676.0); } }
  public RelativeEncoder m_intakeEncoder = m_intake.getEncoder();
  public DigitalInput m_beamBreak = new DigitalInput(IntakeConstants.kBeamBreakPin);
  public NTBoolean m_beamBreakTestSensor = (Robot.isReal() ? null : new NTBoolean(true, testtable, "Intake/BeamBreakTestInput", (val)->{}));

  final DoublePublisher nt_intakeSpeed = intakeTable.getDoubleTopic("speed").publish();
  final DoublePublisher nt_intakeSpeedTarget = intakeTable.getDoubleTopic("speedTarget").publish();
  final DoublePublisher nt_intakeSpeedMax = intakeTable.getDoubleTopic("speedMax").publish();
  public  NTDouble nt_relativePosition = new NTDouble(0.0, testtable,"IntakeRelativePosInch",val ->{ IntakeShooter.getInstance().setIntakeRelativePosition(Units.inchesToMeters(val));}); 
  {nt_relativePosition.resetOnRecv = true;}

  // Beam Break Variables
  private boolean m_beamBroken = false;
  public Debouncer m_beamDebouncer = new Debouncer(IntakeConstants.kBeamBreakDebounceSec, Debouncer.DebounceType.kBoth);

  // Shooter Angle Variables
  double m_angleTargetSpeedRatio = 0;
  double m_angleTarget;

  public boolean m_aimForDistance = false;
  public double m_aimLastDistance;
  public double m_aimVelocityFilt;
  public double m_speakerAngle;
  final DoublePublisher nt_distancefromspkr = shooterTable.getDoubleTopic("actualDistance").publish();
  final DoublePublisher nt_angleDistance = shooterTable.getDoubleTopic("angleDistance").publish();
  final DoublePublisher nt_angleVelocity = shooterTable.getDoubleTopic("angleVelocity").publish();
  final DoublePublisher nt_speakerAngle = shooterTable.getDoubleTopic("speakerAngle").publish();
  final DoublePublisher nt_shooterCurrent = shooterTable.getDoubleTopic("shooterCurrent").publish();
  final DoublePublisher nt_intakeCurrent = shooterTable.getDoubleTopic("intakeCurrent").publish();

  public MyCANSparkMax m_angleMotorLeft = new MyCANSparkMax(ShooterConstants.kCANID_AngleLeft, MotorType.kBrushless);
  public MyCANSparkMax m_angleMotorRight = new MyCANSparkMax(ShooterConstants.kCANID_AngleRight, MotorType.kBrushless);
  public RelativeEncoder m_angleLeftRelativeEncoder;
  public AnalogInput m_angleLeftAnalogEncoder;
  public RelativeEncoder m_angleRightRelativeEncoder;
  public SparkClosedLoopController m_pidAngle;
  public SparkClosedLoopController m_pidAngleRight;
  { if (Robot.isSimulation()){ REVPhysicsSim.getInstance().addSparkMax(m_angleMotorLeft, (float)2.6, (float)5676.0); } }
  { if (Robot.isSimulation()){ REVPhysicsSim.getInstance().addSparkMax(m_angleMotorRight, (float)2.6, (float)5676.0); } }

  final DoublePublisher nt_angle = angleTable.getDoubleTopic("angle").publish();
  final DoublePublisher nt_angleTargetRadians = angleTable.getDoubleTopic("targetRadians").publish();
  final DoublePublisher nt_angleLeftPosition = angleTable.getDoubleTopic("LeftMotorPosition").publish();
  final DoublePublisher nt_angleRightPosition = angleTable.getDoubleTopic("RightMotorPosition").publish();
  public double kAngleReference = NTDouble.create(0, testtable,"ShooterAngle",val ->{ kAngleReference = val; IntakeShooter.getInstance().setShooterAngle(Units.degreesToRadians(val));});
  final DoublePublisher nt_angleVoltage = angleTable.getDoubleTopic("Voltage").publish();
  final DoublePublisher nt_angleLeftCurrent = angleTable.getDoubleTopic("LeftCurrent").publish();
  final DoublePublisher nt_angleRightCurrent = angleTable.getDoubleTopic("RightCurrent").publish();
  final DoublePublisher nt_angleLeftTemp = angleTable.getDoubleTopic("LeftTempF").publish();
  final DoublePublisher nt_angleRightTemp = angleTable.getDoubleTopic("RightTempF").publish();
  final DoublePublisher nt_analogAngle = angleTable.getDoubleTopic("analogAngle").publish();

  // Shooter Variables

  double m_shooterTargetSpeedRatio = 0;
  double m_shooterTargetSpeed = 0;

  private MyCANSparkMax m_shooter = new MyCANSparkMax(ShooterConstants.kCANID_Shooter, MotorType.kBrushless);
  { if (Robot.isSimulation()){ REVPhysicsSim.getInstance().addSparkMax(m_shooter, (float)2.6, (float)5676.0); } }

  private RelativeEncoder m_encoderShoot = m_shooter.getEncoder();

  public SparkClosedLoopController m_pidShooter;

  final DoublePublisher nt_shooterSpeed = shooterTable.getDoubleTopic("speed").publish();
  final DoublePublisher nt_shooterSpeedTarget = shooterTable.getDoubleTopic("speedTarget").publish();
  final DoublePublisher nt_shooterSpeedMax = shooterTable.getDoubleTopic("speedMax").publish();

  final BooleanPublisher nt_beamBreak = intakeTable.getBooleanTopic("beamBreak").publish();

  public void setAngleInverted(){
    m_angleMotorLeft.setInverted(true);
  }

  double m_angleOffset = 0;
  Command m_setupAngle = Commands.sequence(
    Commands.waitUntil(() -> m_angleMotorLeft.restoreFactoryDefaults() == REVLibError.kOk),
    Commands.waitUntil(() -> m_angleMotorRight.restoreFactoryDefaults() == REVLibError.kOk),
    Commands.waitUntil(() -> m_angleMotorLeft.setInverted2(true) == REVLibError.kOk),
    Commands.waitUntil(() -> {m_angleLeftRelativeEncoder = m_angleMotorLeft.getEncoder();
            return m_angleLeftRelativeEncoder != null;}),
    Commands.waitUntil(() -> {m_angleLeftAnalogEncoder = new AnalogInput(ShooterConstants.kAnalogInputpin);
            return m_angleLeftAnalogEncoder != null;}),
    Commands.waitUntil(() -> {m_angleRightRelativeEncoder = m_angleMotorRight.getEncoder();
            return m_angleRightRelativeEncoder != null;}),
    Commands.waitUntil(() -> {m_pidAngle = m_angleMotorLeft.getPIDController();
            return m_pidAngle != null;}),
    Commands.waitUntil(() -> m_angleMotorLeft.setSmartCurrentLimit(ShooterConstants.kAngleCurrentLimit) == REVLibError.kOk),
    Commands.waitUntil(() -> m_angleMotorRight.setSmartCurrentLimit(ShooterConstants.kAngleCurrentLimit) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidAngle.setP(ShooterConstants.kAngleKp) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidAngle.setI(ShooterConstants.kAngleKi) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidAngle.setD(ShooterConstants.kAngleKd) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidAngle.setFF(ShooterConstants.kAngleKff) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidAngle.setOutputRange(-1,1) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidAngle.setIZone(0.15) == REVLibError.kOk),
    Commands.waitUntil(() -> {m_angleOffset = m_angleLeftAnalogEncoder.getValue(); return m_angleOffset > m_angleCalibrationMap.xmin() && m_angleOffset < m_angleCalibrationMap.xmax();}),
    Commands.waitUntil(() -> m_angleLeftRelativeEncoder.setPositionConversionFactor(ShooterConstants.kAngleEncoderPositionFactor) == REVLibError.kOk),
    Commands.waitUntil(() -> m_angleRightRelativeEncoder.setPositionConversionFactor(ShooterConstants.kAngleEncoderPositionFactor) == REVLibError.kOk),
    Commands.waitUntil(() -> m_angleLeftRelativeEncoder.setPosition(m_angleCalibrationMap.get(m_angleOffset)) == REVLibError.kOk),
    Commands.waitUntil(() -> m_angleRightRelativeEncoder.setPosition(m_angleCalibrationMap.get(m_angleOffset)) == REVLibError.kOk),
    Commands.waitUntil(() -> (m_angleMotorRight.follow(m_angleMotorLeft,true)) == REVLibError.kOk),
    Commands.waitUntil(() -> m_angleMotorLeft.burnFlash() == REVLibError.kOk),
    Commands.waitUntil(() -> m_angleMotorRight.burnFlash() == REVLibError.kOk),
    new InstantCommand(() -> m_setupAngleDone = true)
  );

  Command m_setupIntake = Commands.sequence(
    Commands.waitUntil(() -> m_intake.setInverted2(IntakeConstants.kIntakeIsInverted) == REVLibError.kOk),
    Commands.waitUntil(() -> m_intakeEncoder.setVelocityConversionFactor(IntakeConstants.kEncoderVelocityFactor) == REVLibError.kOk),
    Commands.waitUntil(() -> m_intakeEncoder.setPositionConversionFactor(IntakeConstants.kEncoderPositionFactor) == REVLibError.kOk),
    Commands.waitUntil(() -> m_intake.setSmartCurrentLimit(30) == REVLibError.kOk),
    Commands.waitUntil(() -> (m_pidIntake = m_intake.getPIDController()) != null),
    Commands.waitUntil(() -> m_pidIntake.setFeedbackDevice(m_intakeEncoder) == REVLibError.kOk),
    // Velocity Control Gains
    Commands.waitUntil(() -> m_pidIntake.setP(IntakeConstants.kIntakeRollerKp,0) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidIntake.setI(IntakeConstants.kIntakeRollerKi,0) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidIntake.setD(IntakeConstants.kIntakeRollerKd,0) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidIntake.setFF(IntakeConstants.kIntakeRollerKff,0) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidIntake.setOutputRange(-1,1,0) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidIntake.setIZone(0.15,0) == REVLibError.kOk),
    // Position Control Gains
    Commands.waitUntil(() -> m_pidIntake.setP(IntakeConstants.kIntakeRollerPosKp,1) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidIntake.setI(IntakeConstants.kIntakeRollerPosKi,1) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidIntake.setD(IntakeConstants.kIntakeRollerPosKd,1) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidIntake.setFF(IntakeConstants.kIntakeRollerPosKff,1) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidIntake.setOutputRange(-1,1,1) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidIntake.setIZone(0.15,1) == REVLibError.kOk),
    Commands.waitUntil(() -> m_intake.burnFlash() == REVLibError.kOk),
    new InstantCommand(() -> m_setupIntakeDone = true)
    );

  Command m_setupShooter = Commands.sequence(
    Commands.waitUntil(() -> (m_pidShooter = m_shooter.getPIDController()) != null),
    Commands.waitUntil(() -> m_shooter.setSmartCurrentLimit(40) == REVLibError.kOk),
    Commands.waitUntil(() -> m_shooter.setInverted2(ShooterConstants.kShooterIsInverted) == REVLibError.kOk),
    Commands.waitUntil(() -> m_encoderShoot.setVelocityConversionFactor(ShooterConstants.kShooterVelocityFactor) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidShooter.setFeedbackDevice(m_encoderShoot) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidShooter.setP(ShooterConstants.kShooterKp) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidShooter.setI(ShooterConstants.kShooterKi) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidShooter.setD(ShooterConstants.kShooterKd) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidShooter.setFF(ShooterConstants.kShooterKff) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidShooter.setOutputRange(-1,1) == REVLibError.kOk),
    Commands.waitUntil(() -> m_pidShooter.setIZone(0.15) == REVLibError.kOk),
    Commands.waitUntil(() -> m_shooter.burnFlash() == REVLibError.kOk),
    new InstantCommand(() -> m_setupShooterDone = true)
  );

  public void setIntakeSpeedRatio(double ratio) {
    if (!m_setupIntakeDone) return;
    if (ratio > 0) intakeNote.cancel();
    setIntakeSpeed(IntakeConstants.kMaxSpeed * ratio);
  }

  public void setIntakeSpeed(double rpm) {
    if (!m_setupIntakeDone) return;
    if (rpm > 0) intakeNote.cancel();
    m_intakeTargetSpeed = Math.min(IntakeConstants.kMaxSpeed, Math.max(-IntakeConstants.kMaxSpeed, rpm));
    m_pidIntake.setReference(m_intakeTargetSpeed, SparkMax.ControlType.kVelocity, 0);
  }
  public double getIntakeSpeed() {
    if (!m_setupIntakeDone) return 0;
    return m_intakeEncoder.getVelocity();
  }

  public void setIntakeRelativePosition(double positionMeters){
    if (!m_setupIntakeDone) return;
    m_intakeEncoder.setPosition(0); // reset the encoder position to 0
    m_pidIntake.setReference(positionMeters, CANSparkMax.ControlType.kPosition, 1);
  }
  
  public void setIntakePosition(double positionMeters){
    if (!m_setupIntakeDone) return;
    m_pidIntake.setReference(positionMeters, CANSparkMax.ControlType.kPosition, 1);
  }

  public void setShooterSpeedRatio(double ratio) {
    if (!m_setupShooterDone) return;
    setShooterSpeed(ShooterConstants.kMaxSpeed * ratio);
  }

  public void setShooterSpeed(double rpm) {
    if (!m_setupShooterDone) return;
    m_shooterTargetSpeed = Math.min(ShooterConstants.kMaxSpeed, Math.max(-ShooterConstants.kMaxSpeed, rpm));
    m_shooterTargetSpeedRatio = m_shooterTargetSpeed/ShooterConstants.kMaxSpeed;
  }


  public double getShooterSpeed(){
    if (!m_setupShooterDone) return 0;
    return m_encoderShoot.getVelocity();
  }

  public boolean atTargetSpeed(){
    if (!m_setupShooterDone) return false;
    return getShooterSpeed() > ShooterConstants.kAtMaxSpeedPercent * ShooterConstants.kMaxSpeed * m_shooterTargetSpeedRatio;
  }

  public void setShooterAngle(double radians){
    if (!m_setupAngleDone) return;
    radians = Math.max(ShooterConstants.kAngleMin, Math.min(ShooterConstants.kAngleMax, radians));
    m_angleTarget = radians;
    m_pidAngle.setReference(radians, CANSparkMax.ControlType.kPosition);
  }

  public double getAnglePosition() {
    if (!m_setupAngleDone) return 0;
    return (m_angleLeftRelativeEncoder.getPosition() + m_angleRightRelativeEncoder.getPosition())/2.0;
  }

  public double getAngleTarget() {
    return(m_angleTarget);
  }

  public boolean atTargetAngle(){
    if (!m_setupAngleDone) return false;
    return Math.abs(getAnglePosition() - getAngleTarget()) < ShooterConstants.kAngleTolerance;
  }

  // True = Beam Broken, False = Unbroken
  public boolean hasGamePiece() {
    return m_beamBroken;
  }


  double m_aimLastTime = -1;
  public void calcAimDistance(){
    double time = Timer.getFPGATimestamp();

    // Angle vs Distance Calculation
    double distance = VectorUtils.poseDiff(PoseEstimator.getInstance().m_finalPose, PathPlanning.AprilTagAtDistance(AutonomousCommands.speakerTag(),0)).getTranslation().getNorm();
    double angle = ShooterConstants.kAngleDistMap.get(distance); 

    // Velocity Adjustment
    double distance_der = (distance-m_aimLastDistance)/(time-m_aimLastTime);
    m_aimLastTime = time;
    m_aimLastDistance = distance;
    m_aimVelocityFilt = ShooterConstants.kAngleDistDerK * distance_der + (1-ShooterConstants.kAngleDistDerK) * m_aimVelocityFilt;
    m_aimVelocityFilt = Math.min(Math.max(-ShooterConstants.kVelocityMaxAdjust/Math.abs(ShooterConstants.kAngleDistGain), m_aimVelocityFilt),ShooterConstants.kVelocityMaxAdjust/Math.abs(ShooterConstants.kAngleDistGain));

    // Add together angle with velocity adjustment
    m_speakerAngle = angle + ShooterConstants.kAngleDistGain * m_aimVelocityFilt;

    nt_distancefromspkr.set(Units.metersToFeet(distance));
    nt_angleDistance.set(Units.radiansToDegrees(angle));
    nt_angleVelocity.set(Units.radiansToDegrees(ShooterConstants.kAngleDistGain * m_aimVelocityFilt));
    nt_speakerAngle.set(Units.radiansToDegrees(m_speakerAngle));
  }

  public Boolean intakeSequence = false;
  Command intakeNote = (Commands.sequence(
      new InstantCommand(()->{
          setIntakeSpeed(0);
          setShooterSpeedRatio(-1);
          setIntakeRelativePosition(Units.inchesToMeters(IntakeConstants.kRetractDistance));
          setShooterAngle(ShooterConstants.kAnglePreset.Up);
          intakeSequence = true;}),
      Commands.waitUntil(() -> {setIntakeRelativePosition(Units.inchesToMeters(-1.5)); return hasGamePiece() == false;}).withTimeout(2),
      new InstantCommand(()->{setShooterSpeed(0);}),
      Commands.waitUntil(() -> {setIntakeRelativePosition(Units.inchesToMeters(1.0)); return hasGamePiece() == true;}).withTimeout(2)
    ).raceWith(Commands.waitSeconds(5).ignoringDisable(true))).unless(()->!RobotController.isSysActive()).finallyDo(()->{intakeSequence = false;});

  /** Creates a new Intake. */
  boolean m_setupAngleDone = false;
  boolean m_setupIntakeDone = false;
  boolean m_setupShooterDone = false;
  public IntakeShooter() {
    m_setupAngle.ignoringDisable(true).schedule();
    m_setupIntake.ignoringDisable(true).schedule();
    m_setupShooter.ignoringDisable(true).schedule();
  }

  @Override
  public void periodic() {
    
    calcAimDistance();

    if (DriverStation.isAutonomousEnabled() && m_aimForDistance){
      setShooterAngle(m_speakerAngle);
    }

    if (m_setupShooterDone){
      m_pidShooter.setReference(m_shooterTargetSpeed, CANSparkMax.ControlType.kVelocity);
    }

    setShooterAngle(m_angleTarget);
    
    // This method will be called once per scheduler run
    Controller.getAccessoryInstance().Translate();

    boolean beam_trigger = !m_beamBroken;
    if (m_beamBreakTestSensor != null){
      m_beamBroken = m_beamDebouncer.calculate(!m_beamBreakTestSensor.get());
    } else {
      m_beamBroken = m_beamDebouncer.calculate(!m_beamBreak.get());
    }
    // rising edge trigger on beam break sensor
    beam_trigger = beam_trigger && m_beamBroken;

    if (beam_trigger && !intakeSequence && (m_shooterTargetSpeedRatio == 0 || getShooterSpeed() < 50)){
      intakeNote.initialize();
      intakeNote.schedule();
    }

    if (!RobotController.isSysActive() || !m_setupAngleDone) {
      setShooterAngle(getAnglePosition());
      intakeNote.cancel();
      intakeSequence = false;
    }

    nt_shooterSpeed.set(getShooterSpeed());
    nt_shooterCurrent.set(m_shooter.getOutputCurrent());
    nt_shooterSpeedTarget.set(m_shooterTargetSpeedRatio*ShooterConstants.kMaxSpeed);
    nt_shooterSpeedMax.set(ShooterConstants.kMaxSpeed);
    nt_intakeSpeed.set(getIntakeSpeed());
    nt_intakeCurrent.set(m_intake.getOutputCurrent());
    nt_intakeSpeedTarget.set(m_intakeTargetSpeed);
    nt_intakeSpeedMax.set(IntakeConstants.kMaxSpeed);
    nt_beamBreak.set(m_beamBroken);
    nt_angle.set(getAnglePosition());
    nt_angleTargetRadians.set(getAngleTarget());
    if (m_setupAngleDone){
      PoseEstimator.getInstance().m_field.getObject("Intake").setPoses(PoseEstimator.getInstance().m_finalPose, VectorUtils.poseAdd(PoseEstimator.getInstance().m_finalPose, new Pose2d(new Translation2d(Math.cos(getAnglePosition()),PoseEstimator.getInstance().m_finalPose.getRotation()),new Rotation2d(0))));
      nt_angleLeftPosition.set(m_angleLeftRelativeEncoder.getPosition());
      nt_angleRightPosition.set(m_angleRightRelativeEncoder.getPosition());
      nt_angleVoltage.set(m_angleMotorLeft.getBusVoltage());
      nt_angleLeftCurrent.set(m_angleMotorLeft.getOutputCurrent());
      nt_angleRightCurrent.set(m_angleMotorRight.getOutputCurrent());
      nt_angleLeftTemp.set(m_angleMotorLeft.getMotorTemperature() * 1.8 + 32);
      nt_angleRightTemp.set(m_angleMotorRight.getMotorTemperature() * 1.8 + 32);
      nt_analogAngle.set(m_angleLeftAnalogEncoder.getValue());
    }
  }

  double simAngle = 0;
  double simShooterSpeed = 0;
  double simIntakeSpeed = 0;

  @Override
  public void simulationPeriodic() {
    if (m_setupAngleDone){
      simAngle += (m_angleTarget - m_angleLeftRelativeEncoder.getPosition()) * .1;
      m_angleLeftRelativeEncoder.setPosition(simAngle);
      m_angleRightRelativeEncoder.setPosition(simAngle);
    }
   
    if (m_setupShooterDone){
      simShooterSpeed += (m_shooterTargetSpeedRatio*ShooterConstants.kMaxSpeed - m_encoderShoot.getVelocity()) * .1;
    }
    if (m_setupIntakeDone){
      simIntakeSpeed  += (m_intakeTargetSpeed*IntakeConstants.kMaxSpeed - m_intakeEncoder.getVelocity()) * .1;
    }
    
  }
}