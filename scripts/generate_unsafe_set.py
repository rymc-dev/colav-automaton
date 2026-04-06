from colav_unsafe_set import create_unsafe_set
from colav_unsafe_set.objects import DynamicObstacle, Agent

DSF = 40.0
SAFETY_RADIUS = 10.0
agent_position= [0.0, 0.0]
agent_orientation = 0.0
agent_yaw_rate = 0.0 
agent_velocity = 0.0

dynamic_obstacles = []
def main(): 
    def auxiliary_loop():
        agent = Agent(
            position=[agent_position[0], agent_position[1], 0.0],
            orientation=agent_orientation,
            velocity= agent_velocity, 
            yaw_rate=agent_yaw_rate,
            safety_radius=SAFETY_RADIUS
        )
        dyn_obs = []
        for obstacle in dynamic_obstacles: 
            dyn_ob = DynamicObstacle(
                tag="",
                position=(),
                orientation=0.0,
                velocity=0.0,
                yaw_rate=0.0,
                safety_radius=0.0
            ) 
            dyn_obs.append(dyn_ob)

        
        unsafe_set_vertices = create_unsafe_set(agent=agent, dynamic_obstacles=dyn_obs, dsf=DSF)    
        
        
        
if __name__ == '__main__': 
    main()