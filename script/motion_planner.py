class MotionPlanner:
	def __init__ (self, robot, ps):
		self.robot = robot
		self.ps = ps

	def solveBiRRT (self, maxIter = float("inf")):
		self.ps.prepareSolveStepByStep ()
		finished = False
		goalP = [1.57, -1.57, -1.8, 0, 0.8, 0]
		initP = [0.2, -1.57, -1.8, 0, 0.8, 0]
		# In the framework of the course, we restrict ourselves to 2 connected components.
		nbCC = self.ps.numberConnectedComponents ()
		if nbCC != 2:
			raise Exception ("There should be 2 connected components.")

		iter = 0
		while True:
		
		
      #### RRT begin
			solved = False
			newConfigs = list ()
			z=0
			while not solved:
				#print(self.ps.numberConnectedComponents())
				config = self.robot.shootRandomConfig ()
				nearest_config,_ = self.ps.getNearestConfig (config, connectedComponentId=-1)
				valid,path,_ = self.ps.directPath (nearest_config, config, True)
		    
				if not valid :
					l = self.ps.pathLength (path)
					config = self.ps.configAtParam (path, l)
		    
				self.ps.addConfigToRoadmap(config)
				self.ps.addEdgeToRoadmap (nearest_config, config, path, True)

		    ## Try connecting the new nodes together
				for q in newConfigs:
					if q == nearest_config : continue
					
					valid,path,_ = self.ps.directPath (q, config, True)
					if valid:
						self.ps.addEdgeToRoadmap (q, config, path, True)
					if self.ps.numberConnectedComponents() == 1 :
						solved = True
						break
				newConfigs.append(config)
      #### RRT end
      
      
      ## Check if the problem is solved.
			nbCC = self.ps.numberConnectedComponents ()
			if nbCC == 1:
        # Problem solved
				finished = True
				print("\n Init : ", initP)
				print("\n Goal : ", goalP)

				print("\n Current confi added : ", config)
				print("\n Length newConfig : ", len(newConfigs))
				break
			iter = iter + 1
			if iter > maxIter:
				break
		if finished:
			self.ps.finishSolveStepByStep ()
			return self.ps.numberPaths () - 1
		raise RuntimeError("Failed to find a path.")

	def solvePRM (self):
		self.ps.prepareSolveStepByStep ()
    #### PRM begin
    #### PRM end
		self.ps.finishSolveStepByStep ()




"""class MotionPlanner:
	def __init__ (self, robot, ps):
		self.robot = robot
		self.ps = ps

			

	def solveBiRRT (self, maxIter = float("inf")):

		self.ps.prepareSolveStepByStep ()
		finished = False

		# In the framework of the course, we restrict ourselves to 2 connected components.
		nbCC = self.ps.numberConnectedComponents ()
		if nbCC != 2:
			raise Exception ("There should be 2 connected components.")

		goalP = [1.57, -1.57, -1.8, 0, 0.8, 0]
		initP = [0.2, -1.57, -1.8, 0, 0.8, 0]
		#self.ps.addConfigToRoadmap(initP)
		#self.ps.addConfigToRoadmap(goalP)
		'''Algorithm BuildRRT
		    Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq)
		    Output: RRT graph G

		    G.init(qinit)
		    for k = 1 to K do
			qrand ← RAND_CONF()
			qnear ← NEAREST_VERTEX(qrand, G)
			qnew ← NEW_CONF(qnear, qrand, Δq)
			G.add_vertex(qnew)
			G.add_edge(qnear, qnew)
		    return G
		'''
		iter = 0
		
		
		while True:
			#### RRT begin
			solved = False
			newConfigs = []
			while not solved :

				newC = self.robot.shootRandomConfig ()
				nearC,_ = self.ps.getNearestConfig (newC)
				valid,path,_ = self.ps.directPath (nearC, newC, True)
				
				if not valid:
					l = self.ps.pathLength (path)
					config = self.ps.configAtParam (path, l)
				
				self.ps.addConfigToRoadmap(newC)
				self.ps.addEdgeToRoadmap (newC, nearC, path, True)
				
					
				#else: print("collision!")


				

				## Try connecting the new nodes together
				for q in newConfigs:
					if q == nearC : continue
					
					valid,path,_ = self.ps.directPath (q, newC, True)
					if valid:
						self.ps.addEdgeToRoadmap (q, newC, path, 0)
					if self.ps.numberConnectedComponents() == 1 :
						solved = True
						break
				newConfigs.append(newC)
				
			#### RRT end
			## Check if the problem is solved.
			nbCC = self.ps.numberConnectedComponents ()
			if nbCC == 1:
				# Problem solved
				finished = True
				print("\n Init : ", initP)
				print("\n Goal : ", goalP)

				print("\n Current confi added : ", newC)
				print("\n Length newConfig : ", len(newConfigs))
				break
			iter = iter + 1
			if iter > maxIter:
				break
			

			if finished:
				self.ps.finishSolveStepByStep ()
				
				return self.ps.numberPaths () - 1

	def solvePRM (self):
		self.ps.prepareSolveStepByStep ()
		#### PRM begin
		#### PRM end
		self.ps.finishSolveStepByStep ()"""
