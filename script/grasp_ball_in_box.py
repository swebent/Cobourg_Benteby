from math import sqrt
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
from hpp.corbaserver import Client
Client ().problem.resetProblem ()
from manipulation import robot, vf, ps, Ground, Box, Pokeball, PathPlayer, gripperName, ballName

vf.loadEnvironmentModel (Ground, 'ground')
vf.loadEnvironmentModel (Box, 'box')
vf.moveObstacle ('box/base_link_0', [0.3+0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_1', [0.3-0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_2', [0.3, 0.04, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_3', [0.3, -0.04, 0.04, 0, 0, 0, 1])

vf.loadObjectModel (Pokeball, 'pokeball')
robot.setJointBounds ('pokeball/root_joint', [-.4,.4,-.4,.4,-.1,1.,
                                              -1.0001, 1.0001,-1.0001, 1.0001,
                                              -1.0001, 1.0001,-1.0001, 1.0001,])

q1 = [0, -1.57, 1.57, 0, 0, 0, .3, 0, 0.025, 0, 0, 0, 1]

print("Hello, welcome to the best practical ever")

## Create graph
graph = ConstraintGraph (robot, 'graph')

# Create constraints of relative positions 
ballInGripper =  [ 0, .137, 0, 0.5, 0.5, -0.5, 0.5]
gripperAboveBall = [ 0, .237, 0, 0.5, 0.5, -0.5, 0.5]

#Nodes definition

#listNodes = ["grasp_placement", "griper_above_ball", "placement", "grasp", "ball_above_ground"]
listNodes = ["ball_above_ground", "grasp_placement", "grasp", "griper_above_ball", "placement"]
graph.createNode(listNodes)

#Edges defintion
graph.createEdge("grasp_placement"  ,  "griper_above_ball", "move_gripper_up"  , 1, "placement")
graph.createEdge("griper_above_ball",  "grasp_placement"  , "grasp_ball"       , 1, "placement")
graph.createEdge("griper_above_ball",  "placement"        , "move_gripper_away", 1, "placement")
graph.createEdge("placement"        ,  "griper_above_ball", "approach_ball"    , 1, "placement")
graph.createEdge("placement"        ,  "placement"        , "transit"          , 1, "placement")
graph.createEdge("grasp"            ,  "grasp"            , "transfer"         , 1, "grasp")
graph.createEdge("ball_above_ground",  "grasp"            , "take_ball_away"   , 1, "grasp")
graph.createEdge("grasp"            ,  "ball_above_ground", "approach_ground"  , 1, "grasp")
graph.createEdge("ball_above_ground",  "grasp_placement"  , "take_ball_up"     , 1, "grasp")
graph.createEdge("grasp_placement"  ,  "ball_above_ground", "put_ball_down"    , 1, "grasp")

#Constraints defintion
ps.createTransformationConstraint('grasp', gripperName, ballName, 
				  ballInGripper, 6*[True,])
ps.createTransformationConstraint('gripperaboveball', gripperName, 
				  ballName, gripperAboveBall, 6*[True,])

ps.createTransformationConstraint ('placementBallOnGround', '', ballName,
                                   [0, 0, 0.025, 0, 0, 0, 1],
                                   [False, False, True, True, True, False,])

ps.createTransformationConstraint ('placementBallAboveGround', '', ballName,
                                   [0, 0, 0.225, 0, 0, 0, 1],
                                   [False, False, True, True, True, False,])

ps.createTransformationConstraint('placement/complement', '', ballName, 
				  [0,0,0.025,0,0,0,1], 
				  [True, True, False, False, False, True,])

ps.setConstantRightHandSide ('placementBallOnGround', True)
ps.setConstantRightHandSide ('placementBallAboveGround', True)
ps.setConstantRightHandSide ('gripperaboveball', True)
ps.setConstantRightHandSide ('placement/complement', False)

##Adding constraints to the graph

#Nodes
graph.addConstraints (node='placement', constraints = Constraints (numConstraints = ['placementBallOnGround'],))
graph.addConstraints (node='griper_above_ball', constraints = Constraints (numConstraints = ['placementBallOnGround', 'gripperaboveball']))
graph.addConstraints (node='grasp_placement', constraints = Constraints (numConstraints = ['grasp', 'placementBallOnGround']))
graph.addConstraints (node='ball_above_ground', constraints = Constraints (numConstraints = ['grasp', 'placementBallAboveGround']))
graph.addConstraints (node='grasp', constraints = Constraints (numConstraints = ['grasp']))

#Edges
graph.addConstraints(edge='transfer', constraints = Constraints())
graph.addConstraints(edge='move_gripper_up', constraints = Constraints(numConstraints = ['placement/complement']))
graph.addConstraints(edge='grasp_ball', constraints = Constraints(numConstraints = ['placement/complement']))
graph.addConstraints(edge='approach_ball', constraints = Constraints(numConstraints = ['placement/complement']))
graph.addConstraints(edge='move_gripper_away', constraints = Constraints(numConstraints = ['placement/complement']))
graph.addConstraints(edge='transit', constraints = Constraints(numConstraints = ['placement/complement']))
graph.addConstraints(edge='put_ball_down', constraints = Constraints(numConstraints = ['placement/complement']))
graph.addConstraints(edge='take_ball_up', constraints = Constraints(numConstraints = ['placement/complement']))
graph.addConstraints(edge='take_ball_away', constraints = Constraints())
graph.addConstraints(edge='approach_ground', constraints = Constraints(numConstraints = ['placement/complement']))

ps.selectPathValidation ("Discretized", 0.01)
ps.selectPathProjector ("Progressive", 0.1)
graph.initialize ()

## Initial configuration on state 'placement'
res, q_init, error = graph.applyNodeConstraints ('placement', q1)
q2 = q1 [::]
q2 [7] = .2


##Goal configuration on state 'placement'
res, q_goal, error = graph.applyNodeConstraints ('placement', q2)

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

v = vf.createViewer ()
pp = PathPlayer (v)
#v (q1)


## Build relative position of the ball with respect to the gripper
for i in range (100):
  q = robot.shootRandomConfig ()
  res1,q3,err = graph.generateTargetConfig ('approach_ball', q_init, q)
  if res1 and robot.isConfigValid (q3): break;
    
if res:
  robot.setCurrentConfig (q3)
  gripperPose = Transform (robot.getJointPosition (gripperName))
  ballPose = Transform (robot.getJointPosition (ballName))
  gripperGraspsBall = gripperPose.inverse () * ballPose
  gripperAboveBall = Transform (gripperGraspsBall)
  gripperAboveBall.translation [2] += .1
  

print("solving...")
#ps.solve()
print("Done!!...")
#pp(0)


