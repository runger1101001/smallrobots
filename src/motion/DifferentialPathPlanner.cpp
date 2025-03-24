

#include "DifferentialPathPlanner.h"



namespace SmallRobots {


    DifferentialPathPlanner::DifferentialPathPlanner() {
    };

    DifferentialPathPlanner::~DifferentialPathPlanner() {
    };

    void DifferentialPathPlanner::calculate(Pose start, Pose end) {
        Serial.println ("turnRadius: " + (String)turnRadius);
        this->startPose = start;
        this->endPose = end;

        this->S = Vector(startPose.x, startPose.y);
        this->E = Vector(endPose.x, endPose.y);

        this->Sdir = rotation(unitX, startPose.angle);
        this->Edir = rotation(unitX, endPose.angle);    
        // Serial.println ("S: " + (String) S.x + ", " + (String) S.y + ", " + (String) S.z +", angle:" + startPose.angle);
        // Serial.println ("E: " + (String) E.x + ", " + (String) E.y + ", " + (String) E.z +", angle:" + endPose.angle);
        // Serial.println ("Sdir: " + (String) Sdir.x + ", " + (String) Sdir.y + ", " + (String) Sdir.z );
        // Serial.println ("Edir: " + (String) Edir.x + ", " + (String) Edir.y + ", " + (String) Edir.z );

        //RSL = Right, Straight, Left--------------------------------------------------------
        unitV = unit(Sdir);
        cross = crossProduct(unitV, unitZ);
        R1 = S + (cross * turnRadius); //center of first circle to turn Right
        // Serial.println ("unitV: " + (String) unitV.x + ", " + (String) unitV.y + ", " + (String) unitV.z );
        // Serial.println ("cross: " + (String) cross.x + ", " + (String) cross.y + ", " + (String) cross.z );
        // Serial.println ("R1: " + (String) R1.x + ", " + (String) R1.y + ", " + (String) R1.z );

        unitV = unit(Edir);
        cross = crossProduct( unitV, unitZ);
        L2 = E - (cross* turnRadius);   //center of second circle to turn Left
        // Serial.println ("unitV: " + (String) unitV.x + ", " + (String) unitV.y + ", " + (String) unitV.z );
        // Serial.println ("cross: " + (String) cross.x + ", " + (String) cross.y + ", " + (String) cross.z );
        // Serial.println ("L2: " + (String) L2.x + ", " + (String) L2.y + ", " + (String) L2.z );

        //middle point
        a = L2 -  R1;
        A = R1 + (a * 0.5); //only because both circles are the same
        // Serial.println("Middle Point");
        // Serial.println ("a: " + (String) a.x + ", " + (String) a.y + ", " + (String) a.z );
        // Serial.println ("A: " + (String) A.x + ", " + (String) A.y + ", " + (String) A.z );

        //tangent point T1
        float temp = (2*turnRadius)/ magnitude(a);
        // Serial.println("tangent point T1");
        // Serial.println ("temp: " + (String) temp);
        if (temp >=0 && temp <=1) //value region of acos
        {
            alpha = acos(temp);
            // Serial.println ("alpha: " + (String) degrees(alpha));

            u = unit (a) * (turnRadius * cos(alpha)) ;
            unitV = unit(a);
            cross = crossProduct(unitZ, unitV);
            v = cross * (turnRadius*sin(alpha));

            T1= R1 + u + v;
        
            //tangent point T2

            RSL = (A - T1) * 2;

            //Serial.println ("RSL: " + (String) RSL.x + ", " + (String) RSL.y + ", " + (String) RSL.z );

            T2 = T1 + RSL;

            allLength[0] = circularArcLengthCW (Sdir, S, T1, turnRadius) + distance( T1,T2) +  circularArcLengthCCW (Edir, E, T2, turnRadius);
            if (isnan(allLength[0])) allLength[0] = -1;  // should not happen
        } else
        {
            allLength[0] = -1;  
        }                               
        Serial.println (allNames[0]+ ": "+ allLength[0]);

        //RSR = Right, Straight, Right------------------------------------------------------------------
        unitV = unit(Edir);
        cross = crossProduct(unitV, unitZ);
        R2 = E + ( cross*turnRadius);

        //parallel line
        b = R2- R1;

        //tangent point T3
        unitV = unit(b);
        cross = crossProduct(unitZ, unitV);
        T3 = R1 + (cross *turnRadius);

        //tangent point T4
        T4 = T3 + b;

        allLength[1] = circularArcLengthCW (Sdir, S, T3, turnRadius) + distance(T3,T4) +  circularArcLengthCCW (Edir, E, T4, turnRadius);
        if (isnan(allLength[1]))  allLength[1] = -1; //should not happen as RSR and LSL is always possible
        Serial.println (allNames[1]+ ": "+ allLength[1]);

        //RLR = Right, Left, Right ----------------------------------------------------------------------
        float v1 = 4.0*turnRadius*turnRadius;
        float v2 = magnitude(b)*magnitude(b)/4.0;
        if (v1 >=v2 ) //otherwise the circles do not touch and sqrt of negative value
        {
            B = R1 + (b*0.5);
            unitV = unit(b);
            w = (crossProduct(unitV, unitZ)) * ( sqrt(v1 -v2) );
            L3 = B+w;

            //tangent point T5
            T5 = R1 + ((L3 -R1) * 0.5 );
            T6 = R2 + ((L3 -R2) *0.5  );
            sub = T5-R1;
            unitV = unit(sub);
            cross = crossProduct( unitV, unitZ);
            T5dir = unit ( cross );
            allLength[2] = circularArcLengthCW (Sdir, S, T5, turnRadius) +  circularArcLengthCW (T5dir, T5, T6, turnRadius)  +  circularArcLengthCCW (Edir, E, T6, turnRadius);

            if (isnan(allLength[2])) allLength[2] = -1; //should not happen with the check
        }
        else{
                allLength[2] = -1;
        }
        Serial.println (allNames[2]+ ": "+ allLength[2]);

        //--------------------------------------------------------------------------------------------------------------------------------------
        // MIRROWED
        //--------------------------------------------------------------------------------------------------------------------------------------

        //LSR = Left, Straight, Right --------------------------------------------------------

        unitV = unit(Sdir);
        cross = crossProduct(unitV, unitZ);
        L1 = S - ( cross * turnRadius );


        //middle point
        c = R2 - L1;
        C= L1 + (c*0.5); //only because both circles are the same


        //tangent point T7
        temp = (2*turnRadius)/ magnitude(c);
        if (temp >=0 && temp <=1) //value region of acos
        {
            alpha = acos(temp);

            u = unit (c) * ( turnRadius * cos(alpha)) ;
            unitV = unit(c);
            cross = crossProduct(unitV, unitZ);
            v = cross * (turnRadius*sin(alpha));

            T7= L1 + u + v;

            //tangent point T8
            LSR = (C - T7) * 2;
            T8 = T7 + LSR;

            allLength[3] = circularArcLengthCW (Sdir, S, T7, turnRadius) + distance (T7,T8) +  circularArcLengthCCW (Edir, E, T8, turnRadius);
            if (isnan(allLength[3])) allLength[3] = -1; //should not happen anymore
        } else
        {
            allLength[3] = -1;
        }
        Serial.println (allNames[3]+ ": "+ allLength[3]);

        //LSL = Left, Straight, Left ------------------------------------------------------------------

        ////parallel line
        d = L2-L1;

        ////tangent point T9
        unitV = unit(d);
        cross = crossProduct(unitZ, unitV);
        T9 = L1 - (cross * turnRadius);

        ////tangent point T10
        T10 = T9 +d;

        allLength[4] = circularArcLengthCW (Sdir, S, T9, turnRadius) + distance (T9,T10) +  circularArcLengthCCW (Edir, E, T10, turnRadius);
        if (isnan(allLength[4])) allLength[4] = -1; //should not happen as RSR and LSL is always possible
        Serial.println (allNames[4]+ ": "+ allLength[4]);

        //LRL = Left, Right, Left ----------------------------------------------------------------------
        v1 = 4.0*turnRadius*turnRadius;
        v2 =  magnitude(d)*magnitude(d)/4.0;
        if (v1 >=v2 ) //otherwise the circles do not touch and sqrt of negative value
        {
            D = L1 +(d * 0.5);

            unitV = unit(d);
            cross = crossProduct(unitV, unitZ);
            w = cross * sqrt( v1 - v2 ) ;
            R3 = D + w;

            //tangent point T11,12
            T11 = L1+ (R3-L1)*0.5;
            T12 = L2+ (R3-L2)*0.5;

            sub = T11-L1;
            unitV = unit(sub);
            cross = crossProduct( unitV, unitZ);
            T11dir = unit (cross );

            allLength[5] = circularArcLengthCW (Sdir, S, T11, turnRadius) + circularArcLengthCCW (T11dir, T11, T12, turnRadius) +  circularArcLengthCCW (Edir, E, T12, turnRadius);
            if (isnan(allLength[5]) ) allLength[5] = -1;
        } else {
            allLength[5] = -1;
        }
        Serial.println (allNames[5]+ ": "+ allLength[5]);

        getShortestPathIndex ();
    };

    int DifferentialPathPlanner::getShortestPathIndex ()
    {
        float compL = 1000000000;
        for (int i=0; i< 6; i++)
        {
            //if (!isnan(allLength[i])
            if (allLength[i] != -1) //path does not exist
            {
                float l = allLength[i];
                if (l< compL)
                {
                    compL = l;
                    shortestPathIndex= i;
                }
            }
        }
        
        this->name = &allNames[shortestPathIndex];

        Serial.println ("Shortest path is: "+ allNames[shortestPathIndex]+ "  with distance: "+ allLength[shortestPathIndex]+ "   at turnRadius: "+ turnRadius);



        //RSL = Right, Straight, Left--------------------------------------------------------
        if ( allNames[shortestPathIndex].equals("RSL"))
        {
            //set values for shortest path
            // Serial.println("set values for RSL");
            this->arcCenter1 = R1; 
            // Serial.println("arcCenter1: " + (String) arcCenter1.x + " , "+ (String) arcCenter1.y);
            this->arcRadius1 = turnRadius; 
            // Serial.println("arcRadius1: " + (String) arcRadius1);
            this->arcAngle1 = circularArcAngleCW (Sdir, S, T1); this->arcDirName1 ="R";
            // Serial.println("arcAngle1: " + (String) arcAngle1);

            this->arcCenter2 = L2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T2); this->arcDirName2 ="L";

            // Serial.println("arcCenter2: " + (String) arcCenter2.x + " , "+  (String) arcCenter2.y);
            // Serial.println("arcRadius2: " + (String) arcRadius2);
            // Serial.println("arcAngle2: " + (String) arcAngle2);

            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";
            // Serial.println("arcCenter12: " + (String) arcCenter12.x + " , "+  (String) arcCenter12.y);
            // Serial.println("arcRadius12: " + (String) arcRadius12);
            // Serial.println("arcAngle12: " + (String) arcAngle12);

            this->lineStart = T1; this->lineEnd = T2; this->lineLength = distance(lineStart, lineEnd);
            // Serial.println("lineStart: " + (String) lineStart.x +  " , "+ (String) lineStart.y);
            // Serial.println("lineEnd: " + (String) lineEnd.x +  " , "+ (String) lineEnd.y);
            // Serial.println("lineLength: " + (String) lineLength);
            
        }
        
        //RSR = Right, Straight, Right------------------------------------------------------------------
        if ( allNames[shortestPathIndex].equals("RSR"))
        {
            //set values for shortest path
            this->arcCenter1 = R1; this->arcRadius1 = turnRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T3); this->arcDirName1 ="R";

            this->arcCenter2 = R2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T4); this->arcDirName2 ="R";

            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";

            this->lineStart = T3; this->lineEnd = T4;this->lineLength = distance(lineStart, lineEnd);
        }
        //RLR = Right, Left, Right ----------------------------------------------------------------------
        if ( allNames[shortestPathIndex].equals("RLR"))
        {
            //set values for shortest path
            this->arcCenter1 = R1; this->arcRadius1 = turnRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T5);  this->arcDirName1 ="R";

            this->arcCenter2 = R2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T6); this->arcDirName2 ="R";

            this->arcCenter12 = L3; this->arcRadius12 = turnRadius; this->arcAngle12 =  circularArcAngleCW (T5dir, T5, T6); this->arcDirName12 ="L";

            this->lineStart = Vector(); this->lineEnd = Vector();this->lineLength = 0;
        }
        //--------------------------------------------------------------------------------------------------------------------------------------
        // MIRROWED
        //--------------------------------------------------------------------------------------------------------------------------------------

        //LSR = Left, Straight, Right --------------------------------------------------------
        if ( allNames[shortestPathIndex].equals("LSR"))
        {
            //set values for shortest path
            this->arcCenter1 = L1; this->arcRadius1 = turnRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T7); this->arcDirName1 ="L";

            this->arcCenter2 = R2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T8); this->arcDirName2 ="R";

            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";

            this->lineStart = T7; this->lineEnd = T8;this->lineLength = distance(lineStart, lineEnd);
        }
        //LSL = Left, Straight, Left ------------------------------------------------------------------
    if ( allNames[shortestPathIndex].equals("LSL"))
        {
            //set values for shortest path
            this->arcCenter1 = L1; this->arcRadius1 = turnRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T9); this->arcDirName1 ="L";

            this->arcCenter2 = L2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T10); this->arcDirName2 ="L";

            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";

            this->lineStart = T9; this->lineEnd = T10;this->lineLength = distance(lineStart, lineEnd);
        }
        //LRL = Left, Right, Left ----------------------------------------------------------------------
        if ( allNames[shortestPathIndex].equals("LRL"))
        {
            //set values for shortest path
            this->arcCenter1 = L1; this->arcRadius1 = turnRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T11);this->arcDirName1 ="L";

            this->arcCenter2 = L2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T12);this->arcDirName2 ="L";

            this->arcCenter12 = R3; this->arcRadius12 = turnRadius; this->arcAngle12 =  circularArcAngleCW (T11dir, T11, T12); this->arcDirName12 ="R";

            this->lineStart = Vector(); this->lineEnd = Vector();this->lineLength = 0;
        }

        return shortestPathIndex;
    };

    String DifferentialPathPlanner::getShortestPathName()
    {
        return allNames[shortestPathIndex];
    };

    void DifferentialPathPlanner::setPathRadius(float r){
        this->turnRadius=r;
    };


    
}; // namespace SmallRobots