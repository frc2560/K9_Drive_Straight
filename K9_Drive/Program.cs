using System;
using System.Threading;
using Microsoft.SPOT;
using System.Text;


using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;



namespace K9_Drive
{
    public class Program
    {
        /* create a talon */
        static TalonSRX rightSlave = new TalonSRX(4);
        static TalonSRX right = new TalonSRX(3);
        static TalonSRX leftSlave = new TalonSRX(2);
        static TalonSRX left = new TalonSRX(1);
        static PigeonIMU pigeonIMU = new PigeonIMU(rightSlave);
        static StringBuilder stringBuilder = new StringBuilder();

        static CTRE.Phoenix.Controller.GameController _gamepad = null;
        static float kp = 0.5f;
        static float ki = 0.0f;
        static float kd = 0.0f;

        static float integral = 0.0f;
        static float previous_error = 0.0f;

        public static void Main()
        {
            /* loop forever */
            while (true)
            {
                float wanted_angle = 0.0f;
                /* drive robot using gamepad */
                if (_gamepad.GetButton(5))
                {
                    float[] XYZ_Dps = new float[3];
                    pigeonIMU.GetRawGyro(XYZ_Dps);
                    wanted_angle = XYZ_Dps[2];
                    while(_gamepad.GetButton(5))
                        DriveStraight(wanted_angle);
                }
                else
                    Drive();
                /* print whatever is in our string builder */
                Debug.Print(stringBuilder.ToString());
                stringBuilder.Clear();
                /* feed watchdog to keep Talon's enabled */
                CTRE.Phoenix.Watchdog.Feed();
                /* run this task every 20ms */
                Thread.Sleep(20);
            }
        }
        /**
         * If value is within 10% of center, clear it.
         * @param value [out] floating point value to deadband.
         */
        static void Deadband(ref float value)
        {
            if (value < -0.10)
            {
                /* outside of deadband */
            }
            else if (value > +0.10)
            {
                /* outside of deadband */
            }
            else
            {
                /* within 10% so zero it */
                value = 0;
            }
        }
        static void Drive()
        {
            if (null == _gamepad)
                _gamepad = new GameController(UsbHostDevice.GetInstance());

 
            float y = -1 * _gamepad.GetAxis(1);
            float twist = _gamepad.GetAxis(2);

            Deadband(ref y);
            Deadband(ref twist);

            float leftThrot = y + twist;
            float rightThrot = y - twist;

            left.Set(ControlMode.PercentOutput, leftThrot);
            leftSlave.Set(ControlMode.PercentOutput, leftThrot);
            right.Set(ControlMode.PercentOutput, -rightThrot);
            rightSlave.Set(ControlMode.PercentOutput, -rightThrot);


            stringBuilder.Append("\t");
            stringBuilder.Append(y);
            stringBuilder.Append("\t");
            stringBuilder.Append(twist);

        }

        static void DriveStraight(float wanted_angle)
        {
            float[] XYZ_Dps = new float[3];
            pigeonIMU.GetRawGyro(XYZ_Dps);
            float current_angle = XYZ_Dps[2];

            float error = wanted_angle - current_angle;
            integral += (error * .02f);
            float derivative = (error - previous_error) / .02f;
            float rcw = kp * error + ki * integral + kd * derivative;
            previous_error = error;

            float y = -1 * _gamepad.GetAxis(1);
            float twist = rcw;

            float leftThrot = y + twist;
            float rightThrot = y - twist;

            left.Set(ControlMode.PercentOutput, leftThrot);
            leftSlave.Set(ControlMode.PercentOutput, leftThrot);
            right.Set(ControlMode.PercentOutput, -rightThrot);
            rightSlave.Set(ControlMode.PercentOutput, -rightThrot);

            stringBuilder.Append("\t");
            stringBuilder.Append(current_angle);

        }
    }
}