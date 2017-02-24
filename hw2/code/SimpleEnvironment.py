import numpy
import matplotlib.pyplot as pl

import openravepy

import pdb

class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #
        s = self.robot.GetTransform()
        while True:
            config = numpy.multiply(numpy.subtract(upper_limits, lower_limits), numpy.random.random_sample((len(config),))) +  numpy.array(lower_limits)
            q = openravepy.quatFromAxisAngle([0,0,0])
            #Append quaternion with translation to get pose
            pose = numpy.concatenate((q, [config[0], config[1], 0]))
            #Find transform matrix
            Tz = openravepy.matrixFromPose(pose)
            #Post-multiply Tz to robot transform for RELATIVE movement
            #Pre-multiply Tz to robot transform for ABSOLUTE movement
            self.robot.SetTransform(numpy.dot(Tz,self.robot.GetTransform()))
            #Check if 
            if(not((self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetKinBody('conference_table'))))):
                self.robot.SetTransform(s)
                return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        return numpy.linalg.norm(start_config - end_config)
        pass

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        s = self.robot.GetTransform()
        num_steps = 10
        epsilon = 0.00001
        dist = self.ComputeDistance(start_config, end_config)
        step_size = dist/num_steps
        
        direction = (end_config - start_config)/dist
        
        config = start_config + step_size*direction
        lower_limits, upper_limits = numpy.array(self.boundary_limits)

        lower_limits = lower_limits.tolist()
        upper_limits = upper_limits.tolist()
        
        steps = 1
        while True:
            q = openravepy.quatFromAxisAngle([0,0,0])
            #Append quaternion with translation to get pose
            pose = numpy.concatenate((q, [config[0], config[1], 0]))
            #Find transform matrix
            Tz = openravepy.matrixFromPose(pose)
            #Post-multiply Tz to robot transform for RELATIVE movement
            #Pre-multiply Tz to robot transform for ABSOLUTE movement
            self.robot.SetTransform(numpy.dot(Tz,self.robot.GetTransform()))
            #Check if the first step is out of limits, return None
            if((steps==1) and ((config.tolist() < lower_limits) or (config.tolist() > upper_limits))):
                self.robot.SetTransform(s)
                return None
            #Check if the first step collides then return None
            elif((steps==1) and (self.robot.GetEnv().CheckCollision(self.robot, self.robot.GetEnv().GetKinBody('conference_table')))):
                self.robot.SetTransform(s)
                return None
            #If config is out of limits, return previous step
            elif((config.tolist() < lower_limits) or (config.tolist() > upper_limits)):
                self.robot.SetTransform(s)
                return config - step_size*direction
            #If collision occured later, return previous step
            elif(self.robot.GetEnv().CheckCollision(self.robot, self.robot.GetEnv().GetKinBody('conference_table'))):
                self.robot.SetTransform(s)
                return config - step_size*direction

            if numpy.all((numpy.subtract(config, end_config) < epsilon)):
                self.robot.SetTransform(s)
                return end_config
            config += step_size*direction
            steps += 1
        pass

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

