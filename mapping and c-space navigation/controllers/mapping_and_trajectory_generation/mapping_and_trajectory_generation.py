from behaviour_tree import Sequence, Selector, ParallelSelector
from bt_nodes import LoadCspace, CreateCspace, FollowTrajectory, CreateTrajectory
from bt_nodes import context, blackboard, robot
# which blackboard / context to use is configurable for each leaf node:
#   with 'context' and 'blackboard' arguments, shared for all subclasses of LeafNode by default

sink, refrigerator = (0.7, 0.5), (0.9, -1.5)

tree = Sequence([Selector(
                    [LoadCspace(name='check for existing cspace', file='nonexisting_file.npy'), ParallelSelector(
                        [CreateCspace(name='read lidar data into cspace', file='cspace.npy'), FollowTrajectory(name='navigate around table')]
                    )]), 
                CreateTrajectory(target=sink, name='find path to sink'), FollowTrajectory(name='navigate to sink'), 
                CreateTrajectory(target=refrigerator, name='find path to refrigerator'), FollowTrajectory(name='navigate to refrigerator')
                 ])
                 
tree.initialise()

while robot.step(context.timestep) != 1:
    if tree.status == 'RUNNING':  
        tree.update()
    else:
        tree.terminate(tree.status)
        print(f'behaviour tree completed with {tree.status}')
        break