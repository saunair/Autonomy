import numpy
import openravepy
class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.6], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())
            
        #
        # TODO: Generate and return a random configuration
        #
        s = self.robot.GetActiveDOFValues()
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        while True:
            config = numpy.multiply(numpy.subtract(upper_limits, lower_limits), numpy.random.random_sample((len(config),))) +  numpy.array(lower_limits)
            self.robot.SetActiveDOFValues(config)
            if(not((self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetKinBody('conference_table'))))):
                self.robot.SetActiveDOFValues(s)
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
        s = self.robot.GetActiveDOFValues()
        num_steps = 1000
        epsilon = 0.00001
        dist = self.ComputeDistance(start_config, end_config)
        step_size = dist/num_steps
        
        direction = (end_config - start_config)/dist
        
        config = start_config + step_size*direction
        lower_limits, upper_limits = numpy.array(self.robot.GetActiveDOFLimits())

        lower_limits = lower_limits.tolist()
        upper_limits = upper_limits.tolist()
        
        steps = 1
        while True:
            self.robot.SetActiveDOFValues(config)
            #Check if the first step is out of limits, return None
            if((steps==1) and ((config.tolist() < lower_limits) or (config.tolist() > upper_limits))):
                self.robot.SetActiveDOFValues(s)
                print "none"
                return None
            #Check if the first step collides then return None
            elif((steps==1) and (self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetKinBody('conference_table')))):
                self.robot.SetActiveDOFValues(s)
                print "none"
                return None
            #If config is out of limits, return previous step
            elif((config.tolist() < lower_limits) or (config.tolist() > upper_limits)):
                self.robot.SetActiveDOFValues(s)
                print  'out of limits'
                return config - step_size*direction
            #If collision occured later, return previous step
            elif(self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetKinBody('conference_table'))):
                self.robot.SetActiveDOFValues(s)
                print  'return previous step'
                return config - step_size*direction

            if numpy.all((numpy.subtract(config, end_config) < epsilon)):
                self.robot.SetActiveDOFValues(s)
                print 'end config'
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

        delta = 1
        current_time = time.clock()
        leng = len(path)
        while(not(delta > 5)):
            
            g = random.randint(1, leng)
            h = random.randint(1, leng)

            while(g >= h or h==len(path)-1 or g==0):
                g = random.randint(1, leng)
                h = random.randint(1, leng)

            first =  ((np.array(path(g)) +  np.array(path(g-1)))/2).to_list()
            second = ((np.array(path(h)) +  np.array(path(h+1)))/2).to_list()
            
            config = self.Extend(first, second)
            if(config == path(h)):
                

                path(g) = first
                path(h) = second
                redundant_elements = range(g+1, h-1)
                path.remove(redundant_elements)

            delta = time.clock()  - current_time
        return path
