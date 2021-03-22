package org.firstinspires.ftc.teamcode.Subsystems;



public class OdometrySystem {
    private final OdometrySensor x1Sensor;
    private final OdometrySensor x2Sensor;
    private final OdometrySensor y1Sensor;
    private final OdometrySensor y2Sensor;
    private final OdometrySensor angleSensor;
    private double xScale = 1.0;
    private double yScale = 1.0;
    private double angleScale = 1.0;
    private double prevAvgXPos, prevAvgYPos;

    /**
     * @param x1Sensor specifies the first X sensor.
     * @param y1Sensor specifies the first Y sensor.
     * @param y2Sensor specifies the second Y sensor.
     */
    public OdometrySystem(OdometrySensor x1Sensor, OdometrySensor y1Sensor, OdometrySensor y2Sensor){
        this.x1Sensor = x1Sensor;
        this.x2Sensor = null;
        this.y1Sensor = y1Sensor;
        this.y2Sensor = y2Sensor;
        this.angleSensor = null;
    }

    public synchronized void resetOdometry(boolean resetHardware, boolean resetAngle)
    {
        OdometrySensor.Odometry x1Odometry = null;
        OdometrySensor.Odometry x2Odometry = null;
        OdometrySensor.Odometry y1Odometry = null;
        OdometrySensor.Odometry y2Odometry = null;

        if (x1Sensor != null)
        {
            x1Sensor.resetOdometry(resetHardware);
            x1Odometry = x1Sensor.getOdometry();
        }

        if (y1Sensor != null)
        {
            y1Sensor.resetOdometry(resetHardware);
            y1Odometry = y1Sensor.getOdometry();
        }

        if (x2Sensor != null)
        {
            x2Sensor.resetOdometry(resetHardware);
            x2Odometry = x2Sensor.getOdometry();
        }

        if (y2Sensor != null)
        {
            y2Sensor.resetOdometry(resetHardware);
            y2Odometry = y2Sensor.getOdometry();
        }

        if (angleSensor != null && resetAngle)
        {
            angleSensor.resetOdometry(resetHardware);
        }

        prevAvgXPos = averageOdometry(x1Odometry, x2Odometry, true);
        prevAvgYPos = averageOdometry(y1Odometry, y2Odometry, true);
    }   //resetOdometry

//    public synchronized TrcDriveBase.Odometry getOdometryDelta() //TODO finish getOdometryDelta
//    {
//        OdometrySensor.Odometry x1Odometry = x1Sensor != null? x1Sensor.getOdometry(): null;
//        OdometrySensor.Odometry x2Odometry = x2Sensor != null? x2Sensor.getOdometry(): null;
//        OdometrySensor.Odometry y1Odometry = y1Sensor != null? y1Sensor.getOdometry(): null;
//        OdometrySensor.Odometry y2Odometry = y2Sensor != null? y2Sensor.getOdometry(): null;
//        OdometrySensor.Odometry angleOdometry = angleSensor.getOdometry();
//        TrcDriveBase.Odometry odometryDelta = new TrcDriveBase.Odometry();
//
//        double avgXPos = averageOdometry(x1Odometry, x2Odometry, true);
//        double avgYPos = averageOdometry(y1Odometry, y2Odometry, true);
//        double avgXVel = averageOdometry(x1Odometry, x2Odometry, false);
//        double avgYVel = averageOdometry(y1Odometry, y2Odometry, false);
//
//        odometryDelta.position.x = (avgXPos - prevAvgXPos)*xScale;
//        odometryDelta.position.y = (avgYPos - prevAvgYPos)*yScale;
//        odometryDelta.position.angle = (angleOdometry.currPos - angleOdometry.prevPos)*angleScale;
//
//        prevAvgXPos = avgXPos;
//        prevAvgYPos = avgYPos;
//
//        return odometryDelta;
//    }   //getOdometryDelta

    private double averageOdometry(
            OdometrySensor.Odometry odometry1, OdometrySensor.Odometry odometry2, boolean position)
    {
        double value = 0.0;
        int sensorCount = 0;

        if (odometry1 != null)
        {
            value += position? odometry1.currPos: odometry1.velocity;
            sensorCount++;
        }

        if (odometry2 != null)
        {
            value += position? odometry2.currPos: odometry2.velocity;
            sensorCount++;
        }

        return value/sensorCount;
    }   //averageOdometry

    public synchronized Drive.Odometry getOdometryDelta()
    {
        OdometrySensor.Odometry x1Odometry = x1Sensor != null? x1Sensor.getOdometry(): null;
        OdometrySensor.Odometry x2Odometry = x2Sensor != null? x2Sensor.getOdometry(): null;
        OdometrySensor.Odometry y1Odometry = y1Sensor != null? y1Sensor.getOdometry(): null;
        OdometrySensor.Odometry y2Odometry = y2Sensor != null? y2Sensor.getOdometry(): null;
        OdometrySensor.Odometry angleOdometry = angleSensor.getOdometry();
        Drive.Odometry odometryDelta = new Drive.Odometry();

        double avgXPos = averageOdometry(x1Odometry, x2Odometry, true);
        double avgYPos = averageOdometry(y1Odometry, y2Odometry, true);
        double avgXVel = averageOdometry(x1Odometry, x2Odometry, false);
        double avgYVel = averageOdometry(y1Odometry, y2Odometry, false);

        odometryDelta.position.x = (avgXPos - prevAvgXPos)*xScale;
        odometryDelta.position.y = (avgYPos - prevAvgYPos)*yScale;
        odometryDelta.position.angle = (angleOdometry.currPos - angleOdometry.prevPos)*angleScale;
        odometryDelta.velocity.x = avgXVel*xScale;
        odometryDelta.velocity.y = avgYVel*yScale;
        odometryDelta.velocity.angle = angleOdometry.velocity*angleScale;

        prevAvgXPos = avgXPos;
        prevAvgYPos = avgYPos;

        return odometryDelta;
    }   //getOdometryDelta


}
