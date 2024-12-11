import pybullet as p
import time
import pybullet_data
import yaml
from cbs import cbs
import math
import threading
 
 
def create_boundaries(length, width):
    """
        create rectangular boundaries with length and width
 
        Args:
 
        length: integer
 
        width: integer
    """
    for i in range(length):
        p.loadURDF("./final_challenge/assets/cube.urdf", [i, -1, 0.5])
        p.loadURDF("./final_challenge/assets/cube.urdf", [i, width, 0.5])
    for i in range(width):
        p.loadURDF("./final_challenge/assets/cube.urdf", [-1, i, 0.5])
        p.loadURDF("./final_challenge/assets/cube.urdf", [length, i, 0.5])
    p.loadURDF("./final_challenge/assets/cube.urdf", [length, -1, 0.5])
    p.loadURDF("./final_challenge/assets/cube.urdf", [length, width, 0.5])
    p.loadURDF("./final_challenge/assets/cube.urdf", [-1, width, 0.5])
    p.loadURDF("./final_challenge/assets/cube.urdf", [-1, -1, 0.5])
 
 
def create_env(yaml_file):
    """
    Creates and loads assets only related to the environment such as boundaries and obstacles.
    Robots are not created in this function (check `create_turtlebot_actor`).
    """
    with open(yaml_file, 'r') as f:
        try:
            env_params = yaml.load(f, Loader=yaml.FullLoader)
        except yaml.YAMLError as e:
            print(e)
            
    # Create env boundaries
    dimensions = env_params["map"]["dimensions"]
    create_boundaries(dimensions[0], dimensions[1])
 
    # Create env obstacles
    for obstacle in env_params["map"]["obstacles"]:
        p.loadURDF("./final_challenge/assets/cube.urdf", [obstacle[0], obstacle[1], 0.5])
    return env_params
 
 
def create_agents(yaml_file):
    """
    Creates and loads turtlebot agents.
 
    Returns list of agent IDs and dictionary of agent IDs mapped to each agent's goal.
    """
    agent_box_ids = []
    box_id_to_goal = {}
    agent_name_to_box_id = {}
    with open(yaml_file, 'r') as f:
        try:
            agent_yaml_params = yaml.load(f, Loader=yaml.FullLoader)
        except yaml.YAMLError as e:
            print(e)
        
    start_orientation = p.getQuaternionFromEuler([0,0,0])
    for agent in agent_yaml_params["agents"]:
        start_position = (agent["start"][0], agent["start"][1], 0)
        box_id = p.loadURDF("data/turtlebot.urdf", start_position, start_orientation, globalScaling=1)
        agent_box_ids.append(box_id)
        box_id_to_goal[box_id] = agent["goal"]
        agent_name_to_box_id[agent["name"]] = box_id
    return agent_box_ids, agent_name_to_box_id, box_id_to_goal, agent_yaml_params
 
 
def read_cbs_output(file):
    """
        Read file from output.yaml, store path list.
 
        Args:
 
        output_yaml_file: output file from cbs.
 
        Returns:
 
        schedule: path to goal position for each robot.
    """
    with open(file, 'r') as f:
        try:
            params = yaml.load(f, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
    return params["schedule"]
 
 
def checkPosWithBias(Pos, goal, bias):
    """
        Check if pos is at goal with bias
 
        Args:
 
        Pos: Position to be checked, [x, y]
 
        goal: goal position, [x, y]
 
        bias: bias allowed
 
        Returns:
 
        True if pos is at goal, False otherwise
    """
    if(Pos[0] < goal[0] + bias and Pos[0] > goal[0] - bias and Pos[1] < goal[1] + bias and Pos[1] > goal[1] - bias):
        return True
    else:
        return False
 
 
def navigation(agent, goal, schedule):
    """
        Set velocity for robots to follow the path in the schedule.
 
        Args:
 
        agents: array containing the IDs for each agent
 
        schedule: dictionary with agent IDs as keys and the list of waypoints to the goal as values
 
        index: index of the current position in the path.
 
        Returns:
 
        Leftwheel and rightwheel velocity.
    """
    basePos = p.getBasePositionAndOrientation(agent)
    index = 0
    dis_th = 0.4
    while(not checkPosWithBias(basePos[0], goal, dis_th)):
        basePos = p.getBasePositionAndOrientation(agent)
        next = [schedule[index]["x"], schedule[index]["y"]]
        if(checkPosWithBias(basePos[0], next, dis_th)):
            index = index + 1
        if(index == len(schedule)):
            p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=1)
            p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=1)
            break
        x = basePos[0][0]
        y = basePos[0][1]
        Orientation = list(p.getEulerFromQuaternion(basePos[1]))[2]
        goal_direction = math.atan2((schedule[index]["y"] - y), (schedule[index]["x"] - x))
 
        if(Orientation < 0):
            Orientation = Orientation + 2 * math.pi
        if(goal_direction < 0):
            goal_direction = goal_direction + 2 * math.pi
        theta = goal_direction - Orientation
 
        if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
            theta = theta + 2 * math.pi
        elif theta > 0 and abs(theta - 2 * math.pi) < theta:
            theta = theta - 2 * math.pi
 
        current = [x, y]
        distance = math.dist(current, next)
        k1, k2, A = 20, 5, 20
        linear = k1 * math.cos(theta)
        angular = k2 * theta
 
        rightWheelVelocity = linear + angular
        leftWheelVelocity = linear - angular
 
        p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=leftWheelVelocity, force=1)
        p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=rightWheelVelocity, force=1)
        # time.sleep(0.001)
    print(agent, "here")
 
 
def run(agents, goals, schedule):
    """
        Set up loop to publish leftwheel and rightwheel velocity for each robot to reach goal position.
 
        Args:
 
        agents: array containing the boxID for each agent
 
        schedule: dictionary with boxID as key and path to the goal as list for each robot.
 
        goals: dictionary with boxID as the key and the corresponding goal positions as values
    """
    threads = []
    for agent in agents:
        t = threading.Thread(target=navigation, args=(agent, goals[agent], schedule[agent]))
        threads.append(t)
        t.start()
 
    for t in threads:
        t.join()
 
 
# physics_client = p.connect(p.GUI, options='--width=1920 --height=1080 --mp4=multi_3.mp4 --mp4fps=30')
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# Disable tinyrenderer, software (CPU) renderer, we don't use it here
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
 
plane_id = p.loadURDF("plane.urdf")
 
global env_loaded
env_loaded = False
 
# Create environment
env_params = create_env("./final_challenge/env.yaml")
 
# Create turtlebots
agent_box_ids, agent_name_to_box_id, box_id_to_goal, agent_yaml_params = create_agents("./final_challenge/actors.yaml")
 
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera(cameraDistance=5.7, cameraYaw=0, cameraPitch=-89.9,
                                     cameraTargetPosition=[4.5, 4.5, 4])
 
print("Obstacles:", env_params["map"]["obstacles"])
 
# 定义途径点
waypoints = {1: [9, 6], 2:[0,6]}
 
# 第一阶段：起点到途径点
intermediate_agents = [
    {"name": agent["name"], "start": agent["start"], "goal": waypoints[agent["name"]]}
    for agent in agent_yaml_params["agents"]
]
cbs.run(
    dimensions=env_params["map"]["dimensions"],
    obstacles=env_params["map"]["obstacles"],
    agents=intermediate_agents,
    out_file="./final_challenge/cbs_output_part1.yaml",
)
schedule_part1 = read_cbs_output("./final_challenge/cbs_output_part1.yaml")
 
# 第二阶段：途径点到终点
final_agents = [
    {"name": agent["name"], "start": waypoints[agent["name"]], "goal": agent["goal"]}
    for agent in agent_yaml_params["agents"]
]
cbs.run(
    dimensions=env_params["map"]["dimensions"],
    obstacles=env_params["map"]["obstacles"],
    agents=final_agents,
    out_file="./final_challenge/cbs_output_part2.yaml",
)
schedule_part2 = read_cbs_output("./final_challenge/cbs_output_part2.yaml")
 
# 合并路径
full_schedule = {}
for agent_name in schedule_part1:
    full_schedule[agent_name] = schedule_part1[agent_name] + schedule_part2[agent_name]
 
# 替换路径
box_id_to_schedule = {}
for name, value in full_schedule.items():
    box_id_to_schedule[agent_name_to_box_id[name]] = value
 
#cbs_schedule = read_cbs_output("./final_challenge/cbs_output.yaml")
def visualize_path(schedule, obstacles):
    import matplotlib.pyplot as plt
 
    plt.figure()
    for obs in obstacles:
        plt.scatter(obs[0], obs[1], color='red')  # Plot obstacles
 
    for agent, path in schedule.items():
        x, y = zip(*[(step["x"], step["y"]) for step in path])
        plt.plot(x, y, label=f"Agent {agent}")
 
    plt.legend()
    plt.show()
 
visualize_path(box_id_to_schedule, env_params["map"]["obstacles"])
 
# Replace agent name with box id in cbs_schedule
run(agent_box_ids, box_id_to_goal, box_id_to_schedule)
time.sleep(2)
def merge_cbs_outputs(file1, file2, output_file):
    """
    Merge two CBS output YAML files into one.
    
    Args:
   
        file1 (str): Path to the first YAML file (e.g., part1).
        file2 (str): Path to the second YAML file (e.g., part2).
        output_file (str): Path to the output YAML file.
    """
    # 读取两个阶段的路径
    with open(file1, 'r') as f1, open(file2, 'r') as f2:
        part1 = yaml.load(f1, Loader=yaml.FullLoader)
        part2 = yaml.load(f2, Loader=yaml.FullLoader)
 
    # 合并路径
    merged_schedule = {}
    for agent, path1 in part1["schedule"].items():
        # 获取第一阶段路径
        merged_schedule[agent] = []
        for idx, step in enumerate(path1):
            merged_schedule[agent].append({"t": idx, "x": step["x"], "y": step["y"]})
 
        # 获取第二阶段路径，调整时间戳
        offset = len(path1)  # 第二阶段的时间从第一阶段结束时间开始
        for idx, step in enumerate(part2["schedule"][agent]):
            merged_schedule[agent].append({"t": offset + idx, "x": step["x"], "y": step["y"]})
 
    # 保存合并结果
    merged_data = {"cost": part1["cost"] + part2["cost"], "schedule": merged_schedule}
    with open(output_file, 'w') as out_f:
        yaml.dump(merged_data, out_f)
    print(f"Merged file saved to {output_file}")
 
 
# 调用函数
merge_cbs_outputs(
    "./final_challenge/cbs_output_part1.yaml",
    "./final_challenge/cbs_output_part2.yaml",
    "./final_challenge/cbs_output1.yaml"
)