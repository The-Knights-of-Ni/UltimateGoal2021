package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.Locale;

public interface OdometrySensor {
    /**
     * This class implements the generic sensor odometry. It consists of the position as well as velocity info. If
     * the sensor does not support velocity data. This class keeps track of the previous timestamp and position so we
     * can calculate the velocity ourselves.
     */
    public class Odometry
    {
        Object sensor;
        double prevTimestamp;
        double currTimestamp;
        double prevPos;
        double currPos;
        double velocity;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param sensor specifies the sensor object.
         * @param prevTimestamp specifies the timestamp of the previous data.
         * @param currTimestamp specifies the timestamp of the current data.
         * @param prevPos specifies the previous position data.
         * @param currPos specifies the current position data.
         * @param velocity specifies the velocity data. This data may be considered redundant because one can derive
         *                 velocity from (deltaPosition/deltaTime). However, some sensors may support velocity data,
         *                 so this field may contain sensor reported velocity or derived velocity.
         */
        public Odometry(
                Object sensor, double prevTimestamp, double currTimestamp, double prevPos, double currPos,
                double velocity)
        {
            this.sensor = sensor;
            this.prevTimestamp = prevTimestamp;
            this.currTimestamp = currTimestamp;
            this.prevPos = prevPos;
            this.currPos = currPos;
            this.velocity = velocity;
        }   //Odometry

        /**
         * Constructor: Create an instance of the object.
         *
         * @param sensor specifies the sensor object.
         */
        public Odometry(Object sensor)
        {
            this.sensor = sensor;
            this.prevTimestamp = this.currTimestamp = 0.0;
            this.prevPos = this.currPos = this.velocity = 0.0;
        }   //Odometry

        /**
         * This method returns the string representation of the object.
         *
         * @return string representation of the object.
         */
        @Override
        public String toString()
        {
            return String.format(
                    Locale.US, "name=%s,prevTime=%.3f,currTime=%.3f,prevPos=%.1f,currPos=%.1f,vel=%.1f",
                    sensor, prevTimestamp, currTimestamp, prevPos, currPos, velocity);
        }   //toString

        /**
         * This method creates and returns a copy of this odometry.
         *
         * @return a copy of this odometry.
         */
        public Odometry clone()
        {
            return new Odometry(sensor, prevTimestamp, currTimestamp, prevPos, currPos, velocity);
        }   //clone

    }   //class Odometry

    /**
     * This method resets the odometry data and sensor.
     *
     * @param resetHardware specifies true to do a hardware reset, false to do a software reset. Hardware reset may
     *                      require some time to complete and will block this method from returning until finish.
     */
    void resetOdometry(boolean resetHardware);

    /**
     * This method returns a copy of the odometry data. It must be a copy so it won't change while the caller is
     * accessing the data fields.
     *
     * @return a copy of the odometry data.
     */
    Odometry getOdometry();

}
