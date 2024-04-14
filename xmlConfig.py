from xml.dom import minidom
# root = minidom.Document()
  
# problem = root.createElement('Problem') 
# root.appendChild(problem)
  
# productChild = root.createElement('product')
# productChild.setAttribute('name', 'Geeks for Geeks')
  
# problem.appendChild(productChild)
  
# xml_str = root.toprettyxml(indent ="\t") 
# print(xml_str)

"""
<?xml version="1.0" ?>
<Problem solver="sff" optimize="true" smoothing="false" dim="2D">
</Problem>
"""

class XMLconfiguration:
    def __init__(self, dim=2, which="sff", optimize="false"):
        self.root = minidom.Document()
        self.problem = self.root.createElement('Problem')

        self.which = which 
        self.optimize = optimize 
        self.priority_bias = 0 
        if which == "sff":
            self.priority_bias = 0.95

        self.problem.setAttribute("solver", which) 
        self.problem.setAttribute("optimize", optimize) 
        self.problem.setAttribute("smoothing", "false") 
        if dim == 2 :
            self.problem.setAttribute("dim", "2D")
        elif dim == 3:
            self.problem.setAttribute("scale", "10")
            
        self.root.appendChild(self.problem)
        self.miscellaneousSetup()
        pass 

    def miscellaneousSetup(self, dtree=0.5, circum=0.4):
        """ 
        <ObjectDelimiters standard=" " name="_"/>
        <Distances dtree="100" circum="80"/>
        <!-- <Distances dtree="0.5" circum="0.4"/> -->
        <Improvements priorityBias="0.95"/>
        <Thresholds standard="5"/>
        """
        # object delimiters is not required though 
        self.object = self.root.createElement('ObjectDelimiters')
        self.object.setAttribute("standard", " ")
        self.object.setAttribute("name", "_")
        
        self.distances = self.root.createElement('Distances')
        self.distances.setAttribute("dtree", str(dtree))
        self.distances.setAttribute("circum", str(circum))

        self.improvements = self.root.createElement("Improvements")
        self.improvements.setAttribute("priorityBias", str(self.priority_bias))

        self.thresholds = self.root.createElement("Thresholds")
        self.thresholds.setAttribute("standard", "5")

        self.problem.appendChild(self.object)
        self.problem.appendChild(self.distances)
        self.problem.appendChild(self.improvements)
        self.problem.appendChild(self.thresholds)
        pass 

    def setRobot(self, pathToFile):
        """ <Robot file=".//models//robot_small.obj" is_obj="true"/> """
        self.robot = self.root.createElement('Robot')
        self.robot.setAttribute("file", pathToFile)

        if pathToFile[-3:] == "obj":
            self.robot.setAttribute("is_obj", "true")
        else :
            self.robot.setAttribute("is_obj", "false")

        self.problem.appendChild(self.robot)
        pass 

    def setEnvironment(self, pathToFile, collisionResolution=0.01):
        """ 
        <Environment collision="0.01">
            <Obstacle file=".//maps//UniqueSolutionMaze_env.obj" is_obj="true"/>
        </Environment>
        """
        self.env = self.root.createElement('Environment')
        self.env.setAttribute("collision", str(collisionResolution))

        self.obstacle = self.root.createElement("Obstacle")
        self.obstacle.setAttribute("file", pathToFile)

        if pathToFile[-3:] == "obj":
            self.obstacle.setAttribute("is_obj", "true")
        else :
            self.obstacle.setAttribute("is_obj", "false")

        self.env.appendChild(self.obstacle)
        self.problem.appendChild(self.env)
        pass 

    def setTerminals(self, goals):
        """
        <Points>
            <Point coord="[43.0009690005283; -43.62070123660409; 0]"/>
            <Point coord="[-47.25950984226422; -44.166293993959236; 0]"/>
            <Point coord="[-27.5584286318023; 18.98849267088876; 0]"/>
            <Point coord="[1.5732651377668248; 26.141034665664876; 0]"/>
            <!-- <Point coord="[42.55281151136769; 47.192699441349916; 0]"/> -->
            <!-- <Point coord="[43.14058632604015; 37.38599451678651; 0]"/> -->
        </Points>
        """ 
        self.terminals = self.root.createElement('Points')
        for node in goals:
            goal = self.root.createElement('Point')

            valStr = "[" + str(node.state[0]) + "; "  + str(node.state[1]) + "; " 
            try :
                valStr += str(node.state[2])
            except :
                valStr += "0"
            valStr += "]"

            goal.setAttribute("coord", valStr)
            self.terminals.appendChild(goal)

        self.problem.appendChild(self.terminals)
        pass 

    def setBounds(self, bounds):
        """
        <!-- <Range autoDetect="true" /> -->
        <Range autoDetect="false"> 
            <RangeX min="-50" max="50" />
            <RangeY min="-50" max="50" />
            <RangeZ min="0" max="0" />
        </Range>
        """ 
        self.range = self.root.createElement('Range')
        self.range.setAttribute("autoDetect", "false")

        self.rangeX = self.root.createElement('RangeX')
        self.rangeX.setAttribute("min", str(round(bounds.low[0])))
        self.rangeX.setAttribute("max", str(round(bounds.high[0])))

        self.rangeY = self.root.createElement('RangeY')
        self.rangeY.setAttribute("min", str(round(bounds.low[1])))
        self.rangeY.setAttribute("max", str(round(bounds.high[1])))

        try : 
            self.rangeZ = self.root.createElement('RangeZ')
            self.rangeZ.setAttribute("min", str(round(bounds.low[2])))
            self.rangeZ.setAttribute("max", str(round(bounds.high[2])))
        except :
            self.rangeZ = self.root.createElement('RangeZ')
            self.rangeZ.setAttribute("min", "0")
            self.rangeZ.setAttribute("max", "0")

        self.range.appendChild(self.rangeX)
        self.range.appendChild(self.rangeY)
        self.range.appendChild(self.rangeZ)

        self.problem.appendChild(self.range)
        pass

    def setMaxIterations(self, iter):
        """ <MaxIterations value="100000"/> """
        self.iter = self.root.createElement('MaxIterations')
        self.iter.setAttribute("value", str(iter))
        self.problem.appendChild(self.iter)
        pass 

    def setTimeLimit(self, timeLimit=0):
        """  <Time value="3"/> """
        self.timeLimit = self.root.createElement('Time')
        self.timeLimit.setAttribute("value", str(timeLimit))
        self.problem.appendChild(self.timeLimit)
        pass 

    def setSeed(self, seedValue=0):
        """ <Seed value="123"/> """
        self.seed = self.root.createElement('Seed')
        self.seed.setAttribute("value", str(seedValue))
        self.problem.appendChild(self.seed)
        pass 

    def setSaveInstructions(self, folderPath=None, para="params"):
        """
        <Save>
            <Goals file="output//goals.tri" is_obj="false" />
            <Params file="output//params.csv" id="sff_2D_maze"/>
            <TSP file="output//tsp.tsp"/>
            <Tree file="output//tree5_normal_polygons.obj" is_obj="true" everyIteration="0" />
            <RawPath file="output//rawPath5_normal_polygons.tri" is_obj="false" />
        </Save>
        """
        self.saver = self.root.createElement("Save")

        self.params = self.root.createElement("Params")

        if folderPath == None :
            folderPath = "" 
        else :
            folderPath += "/" 

        self.params.setAttribute("file", folderPath + para + ".csv")
        self.params.setAttribute("id", self.which) # any ID to set?

        self.tsp = self.root.createElement("TSP")
        self.tsp.setAttribute("file", folderPath + "tsp.tsp")

        self.saver.appendChild(self.params)
        self.saver.appendChild(self.tsp)
        
        self.problem.appendChild(self.saver)
        pass 


    def output(self, filePath=None):
        self.xml_str = self.root.toprettyxml(indent ="\t") 
        # print(self.xml_str)
        
        try :
            f = open(filePath, "w")
            f.write(self.xml_str)
            f.close()
        except :
            pass 

        return self.xml_str  
