## The frontier-based exploration package

### Subscribed topic 

| topic name | topic type | description |
|------------|------------|---------|
|"/projected_map" | geometry_msgs::PoseStamped | An 2D occupancy map is created by projecting a 3-D octree map to 2-d plane

### Provided service

| service name | service type | description |
|------------|------------|---------|
|"/frontiers_server" | fron::GetPlan | calculate frontiers service

### Note

Provide a service, input the minimum boundary area size, and return all the boundary area centers that meet the requirements

rviz x width 