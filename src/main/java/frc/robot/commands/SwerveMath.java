package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public class SwerveMath {
    double count_swerve;
    double calcWs1Out;
    double calcWs2Out;
    double calcWs3Out;
    double calcWs4Out;
    double[] postOutput = {0,0};
    double reverseEncoderSign;
    boolean shouldReverseSign;
    double waInPos;
    double wsOut;
    double waNew;
    double waTemp;
    double waTempOut;
    double joyStickDriverInRange;
    double[] calcW1OutPost = {0,0};
    double[] calcW2OutPost = {0,0};
    double[] calcW3OutPost = {0,0};
    double[] calcW4OutPost = {0,0};
    double[] swerveW = {0,0,0,0,0,0,0,0};

    public double[] swerveCalculation(double strX1, double rcwX2, double fwdY, double wheelBaseL, double trackWidthW, 
    double gyroReference, double encoderw1, double encoderw2, double encoderw3, double encoderw4){
        if(strX1>0.12 || strX1<(-0.12) || rcwX2>0.12 || rcwX2<(-0.12) || fwdY>0.12 || fwdY<(-0.12)){
            joyStickDriverInRange = 1;
        }else{
            joyStickDriverInRange = 0;
        }

        SmartDashboard.putNumber("strX1", strX1);
        SmartDashboard.putNumber("rcwX2", rcwX2);
        SmartDashboard.putNumber("fwdY", fwdY);
        SmartDashboard.putNumber("joyStickDriverInRange", joyStickDriverInRange);
        
        double calcFwd = fwdY * -1 * joyStickDriverInRange;
        double calcStr = strX1 * joyStickDriverInRange;
        double calcRcw = rcwX2 * joyStickDriverInRange;
        double calcGyroReference = gyroReference/180 *Math.PI;

        double fieldRefTemp = calcFwd * Math.cos(calcGyroReference) + calcStr * Math.sin(calcGyroReference);
        double fieldRefStr = -calcFwd * Math.sin(calcGyroReference) + calcStr * Math.cos(calcGyroReference);
        double fieldRefFwd = fieldRefTemp;

        double calcR = Math.sqrt((wheelBaseL*wheelBaseL) + (trackWidthW*trackWidthW));

        double calcA = fieldRefStr - (calcRcw *(wheelBaseL/calcR));
        double calcB = fieldRefStr + (calcRcw *(wheelBaseL/calcR));
        double calcC = fieldRefFwd - (calcRcw *(trackWidthW/calcR));
        double calcD = fieldRefFwd + (calcRcw *(trackWidthW/calcR));

        double calcWs1 = Math.sqrt(calcB*calcB + calcC*calcC);
        double calcWs2 = Math.sqrt(calcB*calcB + calcD*calcD);
        double calcWs3 = Math.sqrt(calcA*calcA + calcD*calcD);
        double calcWs4 = Math.sqrt(calcA*calcA + calcC*calcC);

        double calcWa1 = Math.atan2(calcB, calcC) * 180/Math.PI;
        double calcWa2 = Math.atan2(calcB, calcD) * 180/Math.PI;
        double calcWa3 = Math.atan2(calcA, calcD) * 180/Math.PI;
        double calcWa4 = Math.atan2(calcA, calcC) * 180/Math.PI;
        
        SmartDashboard.putNumber("WS1", calcWs1);
        SmartDashboard.putNumber("WS2", calcWs2);
        SmartDashboard.putNumber("WS3", calcWs3);
        SmartDashboard.putNumber("WS4", calcWs4);

        SmartDashboard.putNumber("WA1", calcWa1);
        SmartDashboard.putNumber("WA2", calcWa2);
        SmartDashboard.putNumber("WA3", calcWa3);
        SmartDashboard.putNumber("WA4", calcWa4);

        double maxSpeed1 = Math.max(calcWs1, calcWs2);
        double maxSpeed2 = Math.max(calcWs3, calcWs4);
        double maxSpeed = Math.max(maxSpeed1, maxSpeed2);

        if(maxSpeed<=1){
            calcWs1Out = calcWs1;
            calcWs2Out = calcWs2;
            calcWs3Out = calcWs3;
            calcWs4Out = calcWs4;
        }else {
            calcWs1Out = calcWs1/maxSpeed;
            calcWs2Out = calcWs2/maxSpeed;
            calcWs3Out = calcWs3/maxSpeed;
            calcWs4Out = calcWs4/maxSpeed;
        }

        SmartDashboard.putNumber("WS1Out", calcWs1Out);
        SmartDashboard.putNumber("WS2Out", calcWs2Out);
        SmartDashboard.putNumber("WS3Out", calcWs3Out);
        SmartDashboard.putNumber("WS4Out", calcWs4Out);

        //call swervePost Calculation Speed
        calcW1OutPost = swervePost(1, calcWs1Out, calcWa1, encoderw1, false, DriveConstants.swerveGearRatio,false);
        double calcWa1OutPost = swerveW[0] = calcW1OutPost[0];
        double calcWs1OutPost = swerveW[1] = calcW1OutPost[1];
        calcW2OutPost = swervePost(1, calcWs2Out, calcWa2, encoderw2, false, DriveConstants.swerveGearRatio,false);
        double calcWa2OutPost = swerveW[2] = calcW2OutPost[0];
        double calcWs2OutPost = swerveW[3] = calcW2OutPost[1];
        calcW3OutPost = swervePost(1, calcWs3Out, calcWa3, encoderw3, false, DriveConstants.swerveGearRatio,false);
        double calcWa3OutPost = swerveW[4] = calcW3OutPost[0];
        double calcWs3OutPost = swerveW[5] = calcW3OutPost[1];
        calcW4OutPost = swervePost(1, calcWs4Out, calcWa4, encoderw4, false, DriveConstants.swerveGearRatio,false);
        double calcWa4OutPost = swerveW[6] = calcW4OutPost[0];
        double calcWs4OutPost = swerveW[7] = calcW4OutPost[1];

        SmartDashboard.putNumber("WA1Post", calcWa1OutPost);
        SmartDashboard.putNumber("WA2Post", calcWa2OutPost);
        SmartDashboard.putNumber("WA3Post", calcWa3OutPost);
        SmartDashboard.putNumber("WA4Post", calcWa4OutPost);

        SmartDashboard.putNumber("WS1Post", calcWs1OutPost);
        SmartDashboard.putNumber("WS2Post", calcWs2OutPost);
        SmartDashboard.putNumber("WS3Post", calcWs3OutPost);
        SmartDashboard.putNumber("WS4Post", calcWs4OutPost);

        return swerveW;
    }
    public double[] swervePost(double scaleSpeed, double wsIn, double waIn, double encoderIn, boolean reverseEncoder, double gearRatio, boolean reverseSteer){
        double waCalc = waIn/360;
        if(reverseEncoder==true){
            reverseEncoderSign = -1;
        }else{
            reverseEncoderSign = 1;
        }
        double encoderPos = (encoderIn * reverseEncoderSign / gearRatio);
        double encoderPosFloorIQ = Math.floor(encoderPos);
        double encoderPosFloorR = encoderPos + (- 1 * encoderPosFloorIQ);
        boolean shouldReverseOut = shouldReverse(waCalc, encoderPosFloorR);

        if(shouldReverseOut==true)
        {
            wsOut = scaleSpeed * (wsIn * -1);
            if(waCalc<0)
            {
                waNew = waCalc +0.5;
            }
            else
            {
                waNew = waCalc -0.5;
            }
        }
        else
        {
            wsOut = scaleSpeed * (wsIn * 1);
            waNew = waCalc;
        }

        if((waNew - encoderPosFloorR)>0.5)
        {
            waTemp = (waNew + encoderPosFloorIQ) -1;
        }else
        {
            waTemp = (waNew + encoderPosFloorIQ);
        }

        if((waNew - encoderPosFloorR)<(-0.5))
        {
            waTempOut = (waTemp + 1);
        }else
        {
            waTempOut = (waTemp);
        }

        if(reverseSteer == true)
        {
            postOutput[0] = waTempOut * gearRatio * -1;//Angle
        }
        else
        {
            postOutput[0] = waTempOut * gearRatio;//Angle
        }
    
        postOutput[1] = wsOut;//Speed

        return this.postOutput;
    }

    public boolean shouldReverse(double waIn, double caIn){
        if(waIn<0){
            waInPos = waIn + 1;
        }else{
            waInPos = waIn;
        }
        double longDiff = Math.abs(waInPos-caIn);
        double diff = Math.min(longDiff, 1-longDiff);

        if(diff>0.25){
            shouldReverseSign = true;
        }else{
            shouldReverseSign = false;
        }
        return this.shouldReverseSign;
    }

}
