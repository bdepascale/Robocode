using Robocode;
using Robocode.Util;
using System;

using System.Collections.Generic;
using System.Drawing;


namespace CAP4053.Student
{
    public class Target
    {
        public string TargetName;
        public double TargetEnergy;
        public double DistanceTarget;
        public double BearingTargetRadians;
        public double HeadingTargetRadians;
        public double BearingTargetDegrees;
        public double TargetPreviousEnergy;

    }
    public class Point
    {
        public Point(double x, double y)
        {
            X = x;
            Y = y;
        }
        public double X;
        public double Y;
        public double distance(double myX, double myY)
        {
            double x = Math.Pow(this.X - myX, 2.0);
            double y = Math.Pow(this.Y - myY, 2.0);
            return Math.Sqrt(x + y);
        }
    }

    public class PewPewFSM : TeamRobot
    {

        public State CurrentState = State.FindTarget;
        public enum State { FindTarget, Brawl, Ranged }

        public Target CurrentTarget = new Target();

        public int MyDirection = 1;
        List<Point> AntiGravPoints = new List<Point>();
        public override void Run()
        {

            IsAdjustGunForRobotTurn = true;
            IsAdjustRadarForGunTurn = true;
            IsAdjustRadarForRobotTurn = true;
            MaxTurnRate = Rules.MAX_TURN_RATE;
            MaxVelocity = Rules.MAX_VELOCITY;

            // loads up all map endge points before match starts, these do not need re calculated
            for (int i = 0; i < 1200; i++)
            {
                AntiGravPoints.Add(new Point(0, i));
                AntiGravPoints.Add(new Point(i, 0));
                AntiGravPoints.Add(new Point(1200, i));
                AntiGravPoints.Add(new Point(i, 1200));
            }

            BodyColor = Color.BurlyWood;
            BulletColor = Color.BurlyWood;
            RadarColor = Color.DarkSlateBlue;
            GunColor = Color.DarkOliveGreen;
            TurnRadarRightRadians(double.PositiveInfinity);

            while (true)
            {

                TurnRadarRight(360);

            }

        }
        //targetting state bot  searching for a target
        public void FindTarget()
        {
            TurnRadarRight(360);
        }
        // Brawl state bot will get in close to target firing at will and ramming
        public void Brawl()
        {

            SetTurnRight(CurrentTarget.BearingTargetDegrees);
            SetAhead(CurrentTarget.DistanceTarget + 25);
            Execute();

        }
        // Ranged state bot will attempt to stay away from target and fire from a distance, happens as energy gets low
        public void Ranged()
        {

            double ForceX = 0;
            double ForceY = 0;
            // modified version of antigravity
            foreach (Point point in AntiGravPoints)

            {
                double absBearing = Utils.NormalAbsoluteAngle(Math.Atan2(point.X - X, point.Y - Y));
                double distance = point.distance(X, Y);
                ForceX -= Math.Sin(absBearing) / (distance * distance);
                ForceY -= Math.Cos(absBearing) / (distance * distance);
            }

            double Angle = Math.Atan2(ForceX, ForceY);


            if (Math.Abs(Angle - HeadingRadians) < Math.PI / 2)
            {
                SetTurnRightRadians(Utils.NormalRelativeAngle(Angle - HeadingRadians));
                SetAhead(double.PositiveInfinity);
            }
            else
            {
                SetTurnRightRadians(Utils.NormalRelativeAngle(Angle + Math.PI - HeadingRadians));
                SetAhead(double.NegativeInfinity);
            }


            Execute();
        }
        public void Update()
        {
            //sets robot state, if no current target robot will search for one.
            if (CurrentTarget.TargetName == null)
            {
                CurrentState = State.FindTarget;

            }
            // sets state to ranged, robot will use version of antigravity to move away from the enemy and avoiding walls 
            // this happens as energy is lower than 40 but enemy energy must be greater than robot energy else robot continues brawling
            else if (Energy <= 40 && CurrentTarget.TargetEnergy > Energy)
            {
                CurrentState = State.Ranged;

            }
            else
            {
                // brawl state, robot is aggressive, moves in close attempts to ram while firing
                CurrentState = State.Brawl;

            }


            switch (CurrentState)
            {

                case State.FindTarget:
                    FindTarget();
                    break;
                case State.Brawl:
                    Brawl();
                    break;
                case State.Ranged:
                    Ranged();
                    break;
            }


        }
        //updates current target stats on scan
        public void UpdateTarget(ScannedRobotEvent e)
        {

            CurrentTarget.TargetName = e.Name;
            CurrentTarget.TargetEnergy = e.Energy;
            CurrentTarget.BearingTargetRadians = e.BearingRadians;
            CurrentTarget.DistanceTarget = e.Distance;
            CurrentTarget.HeadingTargetRadians = e.HeadingRadians;
            CurrentTarget.BearingTargetDegrees = e.Bearing;
        }
        public override void OnScannedRobot(ScannedRobotEvent e)
        {


            if (IsTeammate(e.Name) == true)
            {
                return;
            }
            if (CurrentTarget.TargetName == null)
            {
                UpdateTarget(e);
                BroadcastMessage(CurrentTarget.TargetName);
            }
            double AbsoluteBearing = HeadingRadians + e.BearingRadians;
            double TargetX = X + Math.Sin(AbsoluteBearing) * e.Distance;
            double TargetY = Y + Math.Cos(AbsoluteBearing) * e.Distance;
            if (CurrentTarget.TargetName == e.Name)
            {

                if (CurrentState == State.Ranged)
                {
                    Dodge(e);
                }
                UpdateTarget(e);

                double FirePower = Math.Min(3.0, Energy);
                double myX = X;
                double myY = Y;


                double TargetHeading = e.HeadingRadians;
                double TargetVelocity = e.Velocity;


                double Time = 0;
                double ArenaHeight = BattleFieldHeight;
                double ArenaWidth = BattleFieldWidth;
                double PredictedX = TargetX;
                double PredictedY = TargetY;
                while ((++Time) * (20.0 - 3.0 * FirePower) <
                      DistanceFromTarget(myX, myY, PredictedX, PredictedY))
                {
                    PredictedX += Math.Sin(TargetHeading) * TargetVelocity;
                    PredictedY += Math.Cos(TargetHeading) * TargetVelocity;
                    if (PredictedX < 18.0
                        || PredictedY < 18.0
                        || PredictedX > ArenaWidth - 18.0
                        || PredictedY > ArenaHeight - 18.0)
                    {
                        PredictedX = Math.Min(Math.Max(18.0, PredictedX),
                                    ArenaWidth - 18.0);
                        PredictedY = Math.Min(Math.Max(18.0, PredictedY),
                                    ArenaHeight - 18.0);
                        break;
                    }
                }
                double FireAngle = Utils.NormalAbsoluteAngle(Math.Atan2(
                    PredictedX - X, PredictedY - Y));

                SetTurnRadarRightRadians(
                    Utils.NormalRelativeAngle(AbsoluteBearing - RadarHeadingRadians));
                SetTurnGunRightRadians(Utils.NormalRelativeAngle(FireAngle - GunHeadingRadians));

                Fire(FirePower);

            }
            // removes enemy previous grav points in preparation for new calculation of updated enemy location
            if (AntiGravPoints.Count > 192)
            {
                AntiGravPoints.RemoveAt(AntiGravPoints.Count - 1);
            }
            AntiGravPoints.Add(new Point(TargetX, TargetY));

            TargetLock();
            Update();
        }


        public double DistanceFromTarget(double myx, double myy, double predictedx, double predictedy)
        {
            double x = Math.Pow(predictedx - myx, 2.0);
            double y = Math.Pow(predictedy - myy, 2.0);
            return Math.Sqrt(x + y);
        }

        // detect when enemy fires based on energy drop, change direction and move from predicted bullet path
        public void Dodge(ScannedRobotEvent e)
        {



            double changeInEnergy =
              CurrentTarget.TargetPreviousEnergy - e.Energy;
            if (changeInEnergy > 0 && changeInEnergy <= 3)
            {

                MyDirection =
                 -MyDirection;
                SetTurnRight(90);
                Ahead((e.Distance / 4 + 36) * MyDirection);
                Execute();


            }
            CurrentTarget.TargetPreviousEnergy = e.Energy;
        }

        public void TargetLock()
        {

            double AbsAngleEnemy = HeadingRadians + CurrentTarget.BearingTargetRadians;
            double TurnRadarBuffer = Math.Min(Math.Atan(36.0 / CurrentTarget.DistanceTarget), Rules.RADAR_TURN_RATE_RADIANS);
            double TurnRadar = Utils.NormalRelativeAngle(AbsAngleEnemy - RadarHeadingRadians);

            if (TurnRadar < 0)
                TurnRadar -= TurnRadarBuffer;
            else
                TurnRadar += TurnRadarBuffer;

            SetTurnRadarRightRadians(TurnRadar);

        }
        public override void OnMessageReceived(MessageEvent evnt)
        {
            CurrentTarget.TargetName = evnt.Message.ToString();
            Update();
        }


        public override void OnHitRobot(HitRobotEvent e)
        {
            if (e.Name == CurrentTarget.TargetName)
            {
                if (GunHeat == 0)
                {
                    TurnGunRightRadians(HeadingRadians + e.BearingRadians - GunHeadingRadians);
                    Fire(3);
                }
                SetBack(50);
                Scan();
                Execute();
            }

            Update();
        }

        public long GetTime()
        {
            return DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;

        }
        // for multiple targets on death of one target, reset and prepare to change state to FindTarget
        public override void OnRobotDeath(RobotDeathEvent evnt)
        {
            if (evnt.Name == CurrentTarget.TargetName)
            {
                CurrentTarget.TargetName = null;
            }
            Update();
        }



    }


}





