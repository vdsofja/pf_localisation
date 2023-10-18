from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random
import random

from time import time


class PFLocaliser(PFLocaliserBase):

       
    def __init__(self):
        #
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.2 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.2 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.2 # Odometry model y axis (side-to-side) noise

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict

        # the num of the particle
        self.numParticle=850

        # store the weight
        self.posesWeight=[]
        for i in range(self.numParticle):
            self.posesWeight.append(0)

        # about the reInit
        self.reInitCount=0
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        

        self.posesWeight=[]
        for i in range(self.numParticle):
            self.posesWeight.append(0)

        # init pose of the robot
        px=initialpose.pose.pose.position.x
        py=initialpose.pose.pose.position.y
        # pz=initialpose.pose.pose.position.z

        ox=initialpose.pose.pose.orientation.x
        oy=initialpose.pose.pose.orientation.y
        oz=initialpose.pose.pose.orientation.z
        ow=initialpose.pose.pose.orientation.w

        if ox==0 and oy==0 and oz==0 and ow==0:
            ow=1

        self.estimatedpose.pose.pose.position.x = px
        self.estimatedpose.pose.pose.position.y = py
        #self.estimatedpose.pose.pose.position.z = pz
        self.estimatedpose.pose.pose.orientation = rotateQuaternion(Quaternion(x=ox,y=oy,z=oz,w=ow),
                                                                    self.INIT_HEADING)
        
        # init particle cloud

        if len(self.particlecloud.poses) == 0:
            for i in range(self.numParticle):
                self.particlecloud.poses.append(Pose())
        else:
            self.particlecloud.poses=[]
            for i in range(self.numParticle):
                self.particlecloud.poses.append(Pose())
        for p in self.particlecloud.poses:
            rnd = random.gauss(0,5)
            
            # ----- Rotate particle according to odometry rotation, plus  noise
            p.orientation = (rotateQuaternion(Quaternion(x=ox,y=oy,z=oz,w=ow),rnd 
                                                  ))
            rnd = random.gauss(0,3)

            p.position.x = (px  +
                            (rnd  * self.ODOM_TRANSLATION_NOISE))
            
            rnd = random.gauss(0,3)
            p.position.y = (py  +
                            (rnd  * self.ODOM_DRIFT_NOISE))

        

        return self.particlecloud
    
          

        


    

        

    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        print(max(self.posesWeight))

        # reInit process
        if max(self.posesWeight) < 12:
            if self.reInitCount >40:
                minx=self.occupancy_map.info.origin.position.x-self.occupancy_map.info.width/2
                minx*=self.occupancy_map.info.resolution
                maxx=self.occupancy_map.info.origin.position.x+self.occupancy_map.info.width/2
                maxx*=self.occupancy_map.info.resolution
                miny=self.occupancy_map.info.origin.position.y-self.occupancy_map.info.height/2
                miny*=self.occupancy_map.info.resolution
                maxy=self.occupancy_map.info.origin.position.y+self.occupancy_map.info.height/2
                maxy*=self.occupancy_map.info.resolution
                self.posesWeight=[]
                for i in range(self.numParticle):
                    self.posesWeight.append(0)




                if len(self.particlecloud.poses) == 0:
                    for i in range(self.numParticle):
                        self.particlecloud.poses.append(Pose())
                else:
                    self.particlecloud.poses=[]
                    for i in range(self.numParticle):
                        self.particlecloud.poses.append(Pose())
                for p in self.particlecloud.poses:
                    px=random.uniform(minx,maxx)
                    py=random.uniform(miny,maxy)
                    # pz=initialpose.pose.pose.position.z

                    ox=0
                    oy=0
                    oz=0
                    ow=1

                    if ox==0 and oy==0 and oz==0 and ow==0:
                        ow=1

                    rnd = random.gauss(0,10)
                    
                    p.orientation = (rotateQuaternion(Quaternion(x=ox,y=oy,z=oz,w=ow),rnd * self.ODOM_ROTATION_NOISE
                                                        ))

                    p.position.x = px  
                                    
                    
                    p.position.y = py
                self.reInitCount=0
            self.reInitCount+=1
        else:
            pass
            # self.reInitCount=0

        # normal process
        
        poseWeight=[]
        if sum(self.posesWeight)<0.1:
            for p in self.particlecloud.poses:
                poseWeight.append(self.sensor_model.get_weight(scan,p))
        else:
            poseWeight=self.posesWeight
        
        # store the pose
        oldPoses=self.particlecloud.poses
        # erase the particle cloud at current
        # self.particlecloud.poses=[]

        # the sum of weight
        total=sum(poseWeight)

        #for i in range(length):
                #self.particlecloud.poses.append(Pose())
        px=0
        py=0
        ox=0
        oy=0
        oz=0
        ow=1

        for p in self.particlecloud.poses:
            count=0
            rnd = random.uniform(0,total)
            for i in range(self.numParticle):
                count+=poseWeight[i]
                if count >=rnd:
                    px=oldPoses[i].position.x
                    py=oldPoses[i].position.y
                    ox=oldPoses[i].orientation.x
                    oy=oldPoses[i].orientation.y
                    oz=oldPoses[i].orientation.z
                    ow=oldPoses[i].orientation.w
                    break
                

            rnd = random.gauss(0,0.05)
            
            # ----- Rotate particle according to odometry rotation, plus  noise
            p.orientation = (rotateQuaternion(Quaternion(x=ox,y=oy,z=oz,w=ow),rnd 
                                                  ))
            rnd = random.gauss(0,0.05)

            p.position.x = (px  +
                            (rnd  ))
            
            rnd = random.gauss(0,0.05)
            p.position.y = (py  +
                            (rnd  ))
                           
        for i in range(self.numParticle):
            self.posesWeight[i] = self.sensor_model.get_weight(scan,self.particlecloud.poses[i])

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        bestWeight = -100
        bestI = 0
        for i in range(self.numParticle):
            if self.posesWeight[i] > bestWeight:
                bestI = i
                bestWeight = self.posesWeight[i]

        return self.particlecloud.poses[bestI]
