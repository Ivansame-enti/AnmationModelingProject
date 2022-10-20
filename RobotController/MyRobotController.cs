using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobotController
{
    public struct MyQuat
    {
        
        public float w;
        public float x;
        public float y;
        public float z;

        public MyQuat(float x2, float y2, float z2, float w2)
        {
            this.w = w2;
            this.x = x2;
            this.y = y2;
            this.z = z2;
        }

        public  static MyQuat Multiply2(MyQuat q1, MyQuat q2)
        {

            float x, y, z, w;
            x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
            y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
            z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
            w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;

            /*a.w* b.w - a.x * b.x - a.y * b.y - a.z * b.z,  // 1
            a.w* b.x + a.x * b.w + a.y * b.z - a.z * b.y,  // i
            a.w* b.y - a.x * b.z + a.y * b.w + a.z * b.x,  // j
            a.w* b.z + a.x * b.y - a.y * b.x + a.z * b.w   // k*/
            //todo: change this so it returns a multiplication:

            return new MyQuat(x, y, z, w).Normalize();
        }
        public MyQuat Normalize()
        {
            //MyVec v = new MyVec(this.x, this.y, this.z) / Math.Sqrt(Math.Pow(this.x, 2) + Math.Pow(this.y, 2) + Math.Pow(this.z, 2));
            float magnitude = (float)Math.Sqrt(Math.Pow(this.x, 2.0f) + Math.Pow(this.y, 2.0f) + Math.Pow(this.z, 2.0f));
            this.x /= magnitude;
            this.y /= magnitude;
            this.z /= magnitude;
            this.w /= magnitude;
            return this;
        }

        public MyQuat Invert()
        {
            this.x *= -1;
            this.y *= -1;
            this.z *= -1;
            return this;
        }





        public static MyQuat FromAngelAxis(MyVec axis, float a)
        {
            axis.Normalize();
            MyVec v = new MyVec(0, 0, 0);
            a = a * (float)Math.PI / 180;

            v.x = axis.x * (float)Math.Sin(a / 2);
            v.y = axis.y * (float)Math.Sin(a / 2);
            v.z = axis.z * (float)Math.Sin(a / 2);
            float w = (float)Math.Cos(a / 2);
            return new MyQuat(v.x, v.y, v.z, w).Normalize();
        }

        public static float GetAngle(MyQuat a)
        {
            return 2 * (float)Math.Acos(a.w);
        }
    }

    public struct MyVec
    {

        public float x;
        public float y;
        public float z;

        public MyVec(float x2, float y2, float z2)
        {
            this.x = x2;
            this.y = y2;
            this.z = z2;
        }

        public MyVec Normalize()
        {
            //MyVec v = new MyVec(this.x, this.y, this.z) / Math.Sqrt(Math.Pow(this.x, 2) + Math.Pow(this.y, 2) + Math.Pow(this.z, 2));
            float magnitude = (float)Math.Sqrt(Math.Pow(this.x, 2.0f) + Math.Pow(this.y, 2.0f) + Math.Pow(this.z, 2.0f));
            this.x /= magnitude;
            this.y /= magnitude;
            this.z /= magnitude;
            return this;
        }
    }






    public class MyRobotController
    {

        
           
        /*private float _initialAngle0 = 69.0f;
        private float _initialAngle1 = 4.37f;
        private float _initialAngle2 = 75.79f;
        private float _initialAngle3 = 33.42f;*/

        private float _initialAngle0 = 74.25f;
        private float _initialAngle1 = -11.387f;
        private float _initialAngle2 = 85.75f;
        private float _initialAngle3 = 41.517f;

        private bool _ej2Activated = false;
        private bool _ej3Activated = false;
        private float _timer = 0;

        /*private float _endAngle0 = 30.66f;
        private float _endAngle1 = 9.22f;
        private float _endAngle2 = 93.03f;
        private float _endAngle3 = 5.59f;*/

        private float _endAngle0 = 41.82f;
        private float _endAngle1 = 2.93f;
        private float _endAngle2 = 64.20f;
        private float _endAngle3 = 41.51f;

        private static MyQuat _sumatoryRot;
        #region public methods



        public string Hi()
        {

            string s = "Pablo Periñan Cutillas, Josep Romera Andreu, Ivan Sales Mendez";
            return s;

        }

        private float Lerp(float firstAngle, float endAngle, float time)
        {
            return firstAngle * (1 - time) + endAngle * time;
        }


        //EX1: this function will place the robot in the initial position

        public void PutRobotStraight(out MyQuat rot0, out MyQuat rot1, out MyQuat rot2, out MyQuat rot3) {

            rot0 = Rotate(NullQ, new MyVec(0f, 1f, 0f), 0f);
            rot1 = Rotate(rot0, new MyVec(1f, 0f, 0f), 0f);
            rot2 = Rotate(rot1, new MyVec(1f, 0f, 0f), 0f);
            _sumatoryRot = rot2;

            //rot3 = Rotate(_localSwing, _ex3TwistAxis, Utils.Lerp(_ex3InitialTwistAngle, _ex3FinalTwistAngle, _t));

            rot3 = Rotate(NullQ, new MyVec(1f, 0f, 0f), 0f);
            rot3 = Rotate(rot3, new MyVec(0f, 1f, 0f), 0f);

            _ej2Activated = false;
            _ej3Activated = false;

            //todo: change this, use the function Rotate declared below
            rot0 = Rotate(NullQ, new MyVec(0f,1f,0f), _initialAngle0);
            rot1 = Rotate(rot0, new MyVec(1f, 0f, 0f), _initialAngle1);
            rot2 = Rotate(rot1, new MyVec(1f, 0f, 0f), _initialAngle2);
            rot3 = Rotate(rot2, new MyVec(1f, 0f, 0f), _initialAngle3);
        }

        public MyQuat M2(MyQuat q1, MyQuat q2)
        {

            return Multiply(q1, q2);
        }

        //EX2: this function will interpolate the rotations necessary to move the arm of the robot until its end effector collides with the target (called Stud_target)
        //it will return true until it has reached its destination. The main project is set up in such a way that when the function returns false, the object will be droped and fall following gravity.


        public bool PickStudAnim(out MyQuat rot0, out MyQuat rot1, out MyQuat rot2, out MyQuat rot3)
        {

            if (!_ej2Activated)
            {
                PutRobotStraight(out rot0, out rot1, out rot2, out rot3);
                _timer = 0;
                _ej2Activated = true;
                _ej3Activated = false;

            }

            if (_timer <=1.0f)
            {
                rot0 = Rotate(NullQ, new MyVec(0f, 1f, 0f), Lerp(_initialAngle0, _endAngle0, _timer));
                rot1 = Rotate(rot0, new MyVec(1f, 0f, 0f), Lerp(_initialAngle1, _endAngle1, _timer));
                rot2 = Rotate(rot1, new MyVec(1f, 0f, 0f), Lerp(_initialAngle2, _endAngle2, _timer));
                rot3 = Rotate(rot2, new MyVec(1f, 0f, 0f), Lerp(_initialAngle3, _endAngle3, _timer));
                _timer += 0.01f;
                return true;
            }

            //todo: remove this once your code works.
            rot0 = Rotate(NullQ, new MyVec(0f, 1f, 0f), Lerp(_initialAngle0, _endAngle0, _timer));
            rot1 = Rotate(rot0, new MyVec(1f, 0f, 0f), Lerp(_initialAngle1, _endAngle1, _timer));
            rot2 = Rotate(rot1, new MyVec(1f, 0f, 0f), Lerp(_initialAngle2, _endAngle2, _timer));
            rot3 = Rotate(rot2, new MyVec(1f, 0f, 0f), Lerp(_initialAngle3, _endAngle3, _timer));

            return false;
        }


        //EX3: this function will calculate the rotations necessary to move the arm of the robot until its end effector collides with the target (called Stud_target)
        //it will return true until it has reached its destination. The main project is set up in such a way that when the function returns false, the object will be droped and fall following gravity.
        //the only difference wtih exercise 2 is that rot3 has a swing and a twist, where the swing will apply to joint3 and the twist to joint4

        public bool PickStudAnimVertical(out MyQuat rot0, out MyQuat rot1, out MyQuat rot2, out MyQuat rot3)
        {
            //todo: add a check for your condition

            if (!_ej3Activated)
            {
                PutRobotStraight(out rot0, out rot1, out rot2, out rot3);
                _timer = 0;
                _ej2Activated = false;
                _ej3Activated = true;
                //resetTwist(rot3);
            }


            if (_timer <= 1.0f)
            {   
                rot0 = Rotate(NullQ, new MyVec(0f, 1f, 0f), Lerp(_initialAngle0, _endAngle0, _timer));
                rot1 = Rotate(rot0, new MyVec(1f, 0f, 0f), Lerp(_initialAngle1, _endAngle1, _timer));
                rot2 = Rotate(rot1, new MyVec(1f, 0f, 0f), Lerp(_initialAngle2, _endAngle2, _timer));
                _sumatoryRot = rot2;

                //rot3 = Rotate(_localSwing, _ex3TwistAxis, Utils.Lerp(_ex3InitialTwistAngle, _ex3FinalTwistAngle, _t));

                rot3 = Rotate(NullQ, new MyVec(1f, 0f, 0f), Lerp(_initialAngle3, _endAngle3, _timer));
                rot3 = Rotate(rot3, new MyVec(0f, 1f, 0f), Lerp(0, 90, _timer));
                
                //rot3 = Rotate(rot2, new MyVec(1f, 0f, 0f), Lerp(_initialAngle3, _endAngle3, _timer));
                _timer += 0.01f;
                
                return true;


            }

            //todo: remove this once your code works.
            rot0 = Rotate(NullQ, new MyVec(0f, 1f, 0f), Lerp(_initialAngle0, _endAngle0, _timer));
            rot1 = Rotate(rot0, new MyVec(1f, 0f, 0f), Lerp(_initialAngle1, _endAngle1, _timer));
            rot2 = Rotate(rot1, new MyVec(1f, 0f, 0f), Lerp(_initialAngle2, _endAngle2, _timer));
            _sumatoryRot = rot2;

            rot3 = Rotate(NullQ, new MyVec(1f, 0f, 0f), Lerp(_initialAngle3, _endAngle3, _timer));
            rot3 = Rotate(rot3, new MyVec(0f, 1f, 0f), Lerp(0, 90, _timer));

            return false;
        }


        public static MyQuat GetSwing(MyQuat rot3)
        {
            MyQuat twist = GetTwist2(rot3);

            MyQuat inversa = twist.Invert();

            MyQuat swing = MyQuat.Multiply2(rot3,inversa);

            return MyQuat.Multiply2(_sumatoryRot, swing);
        }

        public static MyQuat GetTwist(MyQuat rot3)
        {
            return MyQuat.Multiply2(GetSwing(rot3), GetTwist2(rot3));
        }

        public static MyQuat GetTwist2(MyQuat rot3)
        {
            MyQuat q = new MyQuat(0, rot3.y, 0, rot3.w);
            q.Normalize();
            return q;
        }


        #endregion


        #region private and internal methods

        internal int TimeSinceMidnight { get { return (DateTime.Now.Hour * 3600000) + (DateTime.Now.Minute * 60000) + (DateTime.Now.Second * 1000) + DateTime.Now.Millisecond; } }


        private static MyQuat NullQ
        {
            get
            {
                MyQuat a;
                a.w = 1;
                a.x = 0;
                a.y = 0;
                a.z = 0;
                return a;

            }
        }

        internal MyQuat Multiply(MyQuat q1, MyQuat q2) {

            float x, y, z, w;
            x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
            y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
            z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
            w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;

            return new MyQuat(x,y,z,w).Normalize();

        }

        internal MyQuat Rotate(MyQuat currentRotation, MyVec axis, float angle)
        {

            return Multiply(currentRotation, MyQuat.FromAngelAxis(axis, angle));

        }




        //todo: add here all the functions needed

        #endregion






    }
}
