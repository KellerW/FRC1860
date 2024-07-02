package frc.robot;




import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
//import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.NeutralOut;
//import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
//import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.hardware.TalonFX;

//import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.PS5Controller;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.commands.SwerveMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class JavabotsTeleop {
    //Initiate Variables
    double count_auto = 0;
    double count_teleop = 0;
    double count_s =0;
    double swerveWOut[] ={0,0,0,0,0,0,0,0};

    //Variables Motor Steering Swerve 1
    private static final int deviceID = 11;
    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kPTalon, kITalon, kDTalon;

    //Variables Joystick Driver
    PS5Controller m_DriverPs5Controller = new PS5Controller(0);

    //Initiate Pigeon
    PigeonIMU pigeonIMU = new PigeonIMU(5); /* example Pigeon with device ID 0 */

    //Initiate Talon
    TalonFXConfiguration configs = new TalonFXConfiguration();
    TalonFX talonFX1 = new TalonFX(51);
    private final PositionVoltage talonFX1_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    private final VelocityVoltage talonFX1_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    //Rotina Inicio Robo
    public void roboInit(){
        //Initiate Smartdashboard
    SmartDashboard.putNumber("simJoyX", 0);
    SmartDashboard.putNumber("simJoyY", 0);
    SmartDashboard.putNumber("simJoyR", 0);
    SmartDashboard.putNumber("simGyro", 0);
   
    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    //Initialize Falcon Position
    configs.Slot0.kP = 5; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kI = 1;
    configs.Slot0.kD = 1e-3; // A change of 1 rotation per second results in 0.1 volts output
    
    //Initialize Falcon Velocity
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    //configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    //configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    //configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    //onfigs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    //configs.Voltage.PeakForwardVoltage = 8;
    //configs.Voltage.PeakReverseVoltage = -8;
    /* Make sure we start at 0 */
    talonFX1.setPosition(0);
    talonFX1.getConfigurator().apply(configs);

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // PID coefficients Talon Position
    kPTalon = 5; 
    kITalon = 1e-3;
    kDTalon = 1;
    // PID coefficients Talon Velocity
    //kPTalon = 0.11; 
    //kITalon = 0.5;
    //kDTalon = 0.0001; 

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Set Encoder 1", 0);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain Talon", kPTalon);
    SmartDashboard.putNumber("I Gain Talon", kITalon);
    SmartDashboard.putNumber("D Gain Talon", kDTalon);
    }

    //Rotina Teleoperado
    public void Teleop(){
        count_teleop = count_teleop+1;
        System.out.println("Count_Auto:" + count_auto + "Count_Teleop" + count_teleop);
        SwerveMath swervemath = new SwerveMath();
        //double simJoyX = SmartDashboard.getNumber("simJoyX", 0);
        //double simJoyY = SmartDashboard.getNumber("simJoyY", 0);
        //double simJoyR = SmartDashboard.getNumber("simJoyR", 0);
        double simJoyX = m_DriverPs5Controller.getLeftX();
        double simJoyY = m_DriverPs5Controller.getLeftY();
        double simJoyR = m_DriverPs5Controller.getRightX();
        
        
        //get feedback from sterring swerve
        //double fbkEncoderW1 = SmartDashboard.getNumber("Set Encoder 1", 0);

        double fbkEncoderW1 = talonFX1.getPosition().getValue();
        double fbkEncoderW2 = m_encoder.getPosition();
        double fbkEncoderW3 = m_encoder.getPosition();
        double fbkEncoderW4 = m_encoder.getPosition();

        //get feedback from Pigeon
        double [] ypr = new double[3]; pigeonIMU.getYawPitchRoll(ypr);
        if(m_DriverPs5Controller.getR3ButtonPressed()==true){
            pigeonIMU.setYaw(0);
        }
        SmartDashboard.putNumber("Yaw", ypr[0]);
        SmartDashboard.putNumber("Pitch", ypr[1]);
        SmartDashboard.putNumber("Roll", ypr[2]);

        //reference of the robot by Pigeon
        double simGyro = ypr[0]; 

        //call calculation swerve
        swerveWOut = swervemath.swerveCalculation(simJoyX,simJoyR, simJoyY,0.66,0.52,
        simGyro,fbkEncoderW1, fbkEncoderW2, fbkEncoderW3, fbkEncoderW4);

        SmartDashboard.putNumber("WA1output", swerveWOut[0]);
        SmartDashboard.putNumber("WA2output", swerveWOut[2]);
        SmartDashboard.putNumber("WA3output", swerveWOut[4]);
        SmartDashboard.putNumber("WA4output", swerveWOut[6]);

        SmartDashboard.putNumber("WS1output", swerveWOut[1]);
        SmartDashboard.putNumber("WS2output", swerveWOut[3]);
        SmartDashboard.putNumber("WS3output", swerveWOut[5]);
        SmartDashboard.putNumber("WS4output", swerveWOut[7]);

        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);

        // read PID coefficients from SmartDashboard
        double pTalon = SmartDashboard.getNumber("P Gain Talon", 0);
        double iTalon = SmartDashboard.getNumber("I Gain Talon", 0);
        double dTalon = SmartDashboard.getNumber("D Gain Talon", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((pTalon != kPTalon)) {configs.Slot0.kP = pTalon; talonFX1.getConfigurator().apply(configs); kPTalon = pTalon; }
        if((iTalon != kITalon)) {configs.Slot0.kI = iTalon; talonFX1.getConfigurator().apply(configs); kITalon = iTalon; }
        if((dTalon != kDTalon)) {configs.Slot0.kD = dTalon; talonFX1.getConfigurator().apply(configs); kDTalon = dTalon; }
        
        /**
         * PIDController objects are commanded to a set point using the 
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four 
         * parameters:
         *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         *  com.revrobotics.CANSparkMax.ControlType.kPosition
         *  com.revrobotics.CANSparkMax.ControlType.kVelocity
         *  com.revrobotics.CANSparkMax.ControlType.kVoltage
         */

    
        //m_pidController.setReference(swerveWOut[0], CANSparkMax.ControlType.kPosition);//Position: 
        //m_pidController.setReference(rotations, CANSparkMax.ControlType.kVelocity);//Velocity: 
        //m_pidController.setReference(swerveWOut[0], CANSparkMax.ControlType.kPosition);//Percent:

        //Set Talon Position
        talonFX1.setControl(talonFX1_voltagePosition.withPosition(swerveWOut[0]));
        //Set Talon Velocity
        //talonFX1.setControl(talonFX1_voltageVelocity.withVelocity(rotations));
        
        SmartDashboard.putNumber("SetPoint", swerveWOut[0]);
        SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
    }

    
}
