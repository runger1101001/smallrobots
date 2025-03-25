

#include "./MotionController.h"

namespace SmallRobots {

    //------------------------------------------------------------------------------------------------------------------
    //  MotionController
    //------------------------------------------------------------------------------------------------------------------

    MotionController::MotionController(DifferentialKinematics& drive) : kinematics(drive) {
        curPose = Pose();
        addPoseToPath(curPose);
    };

    MotionController::~MotionController() {
    };

    void MotionController::addPoseToPath(Pose p)
    {
        path.push_back(p); //at the end of path, continue current path

    };
    void MotionController::setPoseToReplacePath(Pose p)
    {
        path.clear(); //stop continueing currentpath and st
        addPoseToPath(curPose); //so a path segment is between the start and the new target pose
        path.push_back(p);

    };

    void MotionController::addPoseListToPath(std::vector<Pose> poses){
        path.insert(path.end(), poses.begin(), poses.end());
        
    };
    void MotionController::setPoseListToReplacePath(std::vector<Pose> poses){
        path = poses;
    };

    void MotionController::deletePath()
    {
        path.clear();
    };

    void MotionController::setLoopPath(){
        pathBehaviour = LOOP ; //PATHBEHAVIOURS
    };
    void MotionController::setPausePath(){
        pathBehaviour = PAUSE ; //PATHBEHAVIOURS
    };
    void MotionController::setContinuePath(){
        pathBehaviour =  CONTINUE; //PATHBEHAVIOURS
    };
    void MotionController::setEndPath(){
        pathBehaviour = END ; //PATHBEHAVIOURS
    };
    void MotionController::setRestartPath(){
        pathBehaviour = RESTART; //PATHBEHAVIOURS
    };
    

    void MotionController::setTarget() //get next pose in path, calculate dubin path from current pose and target pose
    {
        // Serial.println ("MotionController::setTarget()");
        // Serial.println("curPathIndex: " + (String) curPathIndex);
        // Serial.println("path queue length: " + (String) path.size());
        targetPose = path[curPathIndex];
        // Serial.println("Current Pose: " + (String)curPose.x + ", " + (String) curPose.y + ", " + (String)degrees( curPose.angle));
        // Serial.println("Target Pose: " + (String)targetPose.x + ", " + (String) targetPose.y + ", " + (String) degrees(targetPose.angle));
        pathPlanner.calculate(curPose, targetPose);
        // Serial.println ("Update path, shortest path: " + pathPlanner.getShortestPathName());

        // Serial.println ("1.) -"+ pathPlanner.arcDirName1+ "- with angle: "+ degrees (pathPlanner.arcAngle1)+ "°, around center: "+ pathPlanner.arcCenter1.x +"," + pathPlanner.arcCenter1.y);
        // if (pathPlanner.arcDirName12.equals ("S")) Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with distance: "+ pathPlanner.lineLength+ ", from start: "+ pathPlanner.lineStart.x + ", " +pathPlanner.lineStart.y+ " to end: "+ pathPlanner.lineEnd.x + ", " + pathPlanner.lineEnd.y);
        // else Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with angle: "+ degrees (pathPlanner.arcAngle12)+ "°, around center: "+ pathPlanner.arcCenter12.x +"," + pathPlanner.arcCenter12.y);
        // Serial.println ("3.) -"+ pathPlanner.arcDirName2+ "- with angle: "+ degrees (pathPlanner.arcAngle2)+ "°, around center: "+ pathPlanner.arcCenter2.x +"," + pathPlanner.arcCenter2.y);

        ICC = pathPlanner.arcCenter1;
        Serial.println ("ICC: " + (String) ICC.x +  " , " + (String) ICC.y);
        curV = Vector (curPose.x, curPose.y);
        // Serial.println("curV: " +  (String) curV.x + " , " + (String) curV.y);

        R = distance(curV, ICC);
        // Serial.println("R: " + (String) R);

        //rest values for next straight move
        if (pathPlanner.arcDirName12.equals ("S")) {
            curDistance= 0;
            lastDistance = pathPlanner.lineLength *2;
            //Serial.println("SET LAST DISTANCE to "+ (String) lastDistance);
        }

    };

    void MotionController::setWheelVelocitiesSeg1() //ARC left or right with minRad
    {
        //Serial.println ("Set wheel velocities segment 1(3) of path.");

        curDirName = pathPlanner.arcDirName1;

        if (pathPlanner.arcDirName1.equals ("L")) {
          kinematics.move(vRobot, R); 
          targetAngle = pathPlanner.arcAngle1 + curPose.angle ;
        } else if (pathPlanner.arcDirName1.equals ("R")){
          kinematics.move(vRobot, -R); 
          targetAngle = -pathPlanner.arcAngle1 + curPose.angle ;
        }
        //Serial.println ("targetAngle: "+ String (degrees(targetAngle))+ " °");
    };
    void MotionController::setWheelVelocitiesSeg2() //STRAIGHT or left or right ARC
    {

        Serial.println ("Set wheel velocities segment 2(3) of path.");

        curDirName = pathPlanner.arcDirName12;

        if (pathPlanner.arcDirName12.equals ("S")) {

            // Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with distance: "+ pathPlanner.lineLength+ "+ from start: "+ pathPlanner.lineStart.x + ", " +  pathPlanner.lineStart.y + " to end: "+ pathPlanner.lineEnd.x + ", " + pathPlanner.lineEnd.y);
            targetPose = Pose() ;
            targetPose.x = pathPlanner.lineEnd.x;
            targetPose.y = pathPlanner.lineEnd.y;
            targetPose.angle = curPose.angle;
            // GO Straight
            kinematics.move(vRobot);

            //rest values for next straight move
      
            curDistance= 0;
            lastDistance = pathPlanner.lineLength *2;
            // Serial.println("SET LAST DISTANCE to "+ (String) lastDistance);
    

          } else
          { //R or L

            // Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with angle: "+ degrees (pathPlanner.arcAngle12)+ "°, around center: "+ pathPlanner.arcCenter12.x + ", " + pathPlanner.arcCenter12.y);

            //Serial.println("current robot angle:  "+ String(degrees (curPose.angle))+ " °");

            ICC = pathPlanner.arcCenter12; //OR ? in theory the same: Vector(curPose.x - R * sin(curPose.angle), curPose.y + R* cos(curPose.angle) );
            R = pathPlanner.turnRadius;//OR ? in theory the same:  distance(Vector (curPose.x, curPose.y), ICC);
            // curV = Vector (curPose.x, curPose.y);
            // R = distance(curV, ICC);

            if (pathPlanner.arcDirName12.equals ("L")) {
                kinematics.move(vRobot, R); 
                targetAngle = pathPlanner.arcAngle12 + curPose.angle ;
            } else if (pathPlanner.arcDirName12.equals ("R")) {
                kinematics.move(vRobot, -R); 
                targetAngle = -pathPlanner.arcAngle12 + curPose.angle ;
            }
          }

    };
    void MotionController::setWheelVelocitiesSeg3() //ARC left or right
    {

        // Serial.println ("Set wheel velocities segment 3(3) of path.");

        // Serial.println ("3.) -"+ pathPlanner.arcDirName2+ "- with angle: "+ degrees (pathPlanner.arcAngle2)+ "°, around center: "+ pathPlanner.arcCenter2.x + ", " + pathPlanner.arcCenter2.y);
        // Serial.println("current robot angle:  "+ String(degrees (curPose.angle))+ " °");

        ICC = pathPlanner.arcCenter2;//OR ? in theory the same: Vector(curPose.x - R * sin(curPose.angle), curPose.y + R* cos(curPose.angle) );
        R = pathPlanner.turnRadius;//OR ? in theory the same:  distance(Vector (curPose.x, curPose.y), ICC);
        // curV = Vector (curPose.x, curPose.y);
        // R = distance(curV, ICC);


        curDirName = pathPlanner.arcDirName2;

        if (pathPlanner.arcDirName2.equals ("L")) {
            kinematics.move(vRobot, R); 
            targetAngle = pathPlanner.arcAngle2 + curPose.angle ;
        } else if (pathPlanner.arcDirName2.equals ("R")){
            kinematics.move(vRobot, -R);
            targetAngle = -pathPlanner.arcAngle2 + curPose.angle ;
        }

        //Serial.println ("targetAngle: " + String (degrees(targetAngle)) + " °");

    };

    void MotionController::stop(){
        kinematics.stop();
    };

    void MotionController::stopMoving(){
        kinematics.stop();
    };

    void MotionController::enableMotors(){
        kinematics.start();
    };

    void MotionController::setRobotVelocity( float _vRobot){ //in mm/s
        //make sure that the sign of new velocity is the same as in current movement
        float speed = kinematics.getCurRobotSpeed();
        // if (speed >= 0)vRobot = abs( _vRobot );
        // else vRobot = - abs(_vRobot);
        vRobot =_vRobot;
        float radius = kinematics.getCurRobotRadius();
        kinematics.move(vRobot, radius);
    };

    void MotionController::setPathRadius(float _radius){ //this is only for the next move command from dubin path
        pathPlanner.setPathRadius(_radius);//in mm
    }; 

    void MotionController::setCurPose(Pose pose){
        curPose = pose;
        //Serial.println ("curPose : " + (String) curPose.x+ ", " +(String) curPose.y+ ", " + (String) degrees(curPose.angle)) ;

    };


    bool MotionController::loopPath()
    {
        bool loop= true;
        curPathIndex++;
        if (curPathIndex >= path.size()) curPathIndex= 0; 
        if (path.size() ==1) loop = false;
        return loop;
    };

    bool MotionController::checkIfArrived(){
        bool arrived = false;
        
        if (curDirName.equals("S")){
            Vector curV = Vector(curPose.x, curPose.y);
            Vector tarV = Vector (targetPose.x, targetPose.y);
            //Serial.println("curV:" + String(curV.x) + ", " + (String) curV.y);
            //Serial.println("tarV:" + String(tarV.x) + ", " + (String) tarV.y);
            curDistance = distance( curV, tarV );
            //Serial.println("distance: " + (String) curDistance + "< lastDistance: " + (String) lastDistance);
        } else
        {
            //Serial.println ("targetAngle - curPose.angle :"
            // + String (degrees(targetAngle))+ " ° - "
            // + String (degrees(curPose.angle ))+ " ° = "
            // + String ( degrees (targetAngle - curPose.angle)) + " °"
            // );
            
        }
        if (
          //(curDirName.equals ("L") && targetAngle - curPose.angle  <= arriveAngleDistance )
          (curDirName.equals ("L") && targetAngle - curPose.angle  <= arriveAngleDistance )
          ||
          (curDirName.equals ("R")  && targetAngle -  curPose.angle >= arriveAngleDistance) 
          ||
          //(curDirName.equals ("S")  && curDistance <= arriveDistance) ) //TODO OR CHECK lastDistance > distance, if that changes, then overshoot!, or slowdown when close to target?
          (curDirName.equals ("S")  && curDistance > lastDistance) 
        )
        {
            arrived = true;
            // Serial.println("ARRIVED");
           //reset distances when next pose path is calculated
        }
    
        if (curDirName.equals ("S"))  lastDistance = curDistance;
         
        return arrived;
    };

}; // namespace SmallRobots