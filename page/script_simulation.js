/*MQTT通信，数据收发及路径绘制部分 */
let isRunning = false;// 控制运行状态的标志
let algorithmflag = '';//后端通信的算法选择标志位，已弃用
let mapflag = 0;//后端通信的地图选择标志位
// 标志变量，初始时不允许自动加载地图
let isMapLoaded = false;
function runExperiment() {
    const animationContainer = document.getElementById('animation-container');
    // 获取需要保留的元素
    const mapContainer = document.getElementById('mapContainer');
    let hasDrawn = false;
    var selectedAlgorithm = document.getElementById('algorithmSelect').value;
    var startCoord = document.getElementById('startCoord').value.split(',').map(Number);
    console.log("起点: ", startCoord);
    var endCoord = document.getElementById('endCoord').value.split(',').map(Number);
    console.log(" 终点: ", endCoord);
    
    console.log("算法标志位: ", algorithmflag);
    console.log("地图标志位: ", mapflag);
    //var resultDetails = document.getElementById('resultDetails');
    //var experimentCanvas = document.getElementById('experimentCanvas');
    var runButton = document.getElementById('runButton');
    animationContainer.innerHTML = '';
    animationContainer.appendChild(mapContainer);
    
    if(isRunning){
        // 或者，更简单的方法是先清空所有子元素，然后再把mapContainer加回去
        
        runButton.textContent = '运行实验';
        isRunning = false;
        
        // document.getElementById('startCoord').value = '';
        // document.getElementById('endCoord').value = '';
        return;
    }
    else{
        isRunning = true;
        runButton.textContent = '重置实验';
            
        // 创建 MQTT 客户端
        const client = new Paho.Client("192.168.1.111", 8083, "clientId-" + Math.random());

        // 设置回调函数
        client.onConnectionLost = onConnectionLost;
        client.onMessageArrived = onMessageArrived;

        // 连接到 MQTT Broker
        client.connect({
            onSuccess: onConnect,
            onFailure: onFailure
        });

        // 连接成功时的回调
        function onConnect() {
            console.log("Connection Success");
            //随机生成话题名
            // 生成一个1到10000之间的随机数
            var randomNumber = Math.floor(Math.random() * 10000) + 1;

            // 将随机数转换为字符串，并生成话题名
            var topic_name = "planning" + randomNumber;

            console.log(topic_name);
            // 将数组打包为 JSON 对象
            var dataToSend = {
                '起点': startCoord,
                '目标点': endCoord,
                'code': selectedAlgorithm,
                'map': mapflag,
                'topic_pub':topic_name
            };

            // 将 JSON 对象转换为字符串
            const jsonString = JSON.stringify(dataToSend);

            // 发送 JSON 数据到指定主题
            const topic = "/clientinfo";
            const localmsg = new Paho.Message(jsonString);
            localmsg.destinationName = topic;
            client.send(localmsg);
            console.log("Local Message Send!: ", jsonString);
            client.subscribe(topic_name);
        }

        // 连接失败时的回调
        function onFailure(responseObject) {
            console.log("Connection Failed: " + responseObject.errorMessage);
        }

        // 连接丢失时的回调
        function onConnectionLost(responseObject) {
            if (responseObject.errorCode !== 0) {
                console.log("Connection Lost: " + responseObject.errorMessage);
            }
        }

        

        // 消息到达时的回调
        function onMessageArrived(message) {

            if (hasDrawn) return;

            console.log("收到的完整消息: ", message.payloadString);
            if (message.payloadString) {
                try {
                    const data = JSON.parse(message.payloadString);
                    console.log("解析后的数据: ", data);
                    
                    // 解析 JSON 数据
                    const {障碍物, 车辆行使路径,参考线,path} = data;
                    if(path == 0){
                        alert("路径规划失败，请重新指定起点终点！");
                    }
                    else if(path == 1){
                        // 初始化 Two.js
                    // 创建并添加 canvas 元素
                    const elem = document.getElementById('animation-container');
                    

                    // 初始化 Two.js
                    
                    


                    const params = { width: 729, height: 540 };
                    const two = new Two(params).appendTo(elem);

                    two.clear();
                    
                    // 缩放因子
                    const scaleFactor = 27;
                

                    // 绘制起点
                    const startCircle = two.makeCircle(startCoord[0] * scaleFactor+13, startCoord[1] * scaleFactor+13, 13);
                    startCircle.fill = 'red';

                    // 绘制目标点
                    const targetCircle = two.makeCircle(endCoord[0] * scaleFactor+13, endCoord[1] * scaleFactor+13, 13);
                    targetCircle.fill = 'green';

                    //绘制障碍物
                    障碍物.forEach(([x, y]) => {
                        const obstacleCircle = two.makeRectangle(y * scaleFactor+13, x * scaleFactor+13, 27, 27);
                        obstacleCircle.linewidth = 0;
                        obstacleCircle.fill = 'blue';
                    });

                    // 动态绘制车辆路径
                    let currentIndex = 0;
                    const vehicle = two.makeCircle(车辆行使路径[0][0] * scaleFactor+13, 车辆行使路径[0][1] * scaleFactor+13, 13);
                    //const vehicle = two.makeRectangle(车辆行使路径[0][0] * scaleFactor+15, 车辆行使路径[0][1] * scaleFactor+15, 20,30);
                    vehicle.fill = 'yellow';

                    two.bind('update', function (frameCount) {
                        if (currentIndex < 车辆行使路径.length - 1) {
                            const [x1, y1] = 车辆行使路径[currentIndex];
                            const [x2, y2] = 车辆行使路径[currentIndex + 1];
                            vehicle.translation.set(x2 * scaleFactor+13, y2 * scaleFactor+13);

                            // 绘制线段
                            const line = two.makeLine(x1 * scaleFactor+13, y1 * scaleFactor+13, x2 * scaleFactor+13, y2 * scaleFactor+13);
                            line.stroke = 'black';
                            line.linewidth = 2;

                            currentIndex++;
                        }
                    }).play();
                    if(selectedAlgorithm == 'frenet'){
                        参考线.forEach((point, index) => {
                            if (index > 0) {
                                // 计算起点和终点的坐标
                                const [x1, y1] = 参考线[index - 1];
                                const [x2, y2] = point;
                        
                                // 应用缩放因子并偏移13（如果需要）
                                const scaledX1 = (x1 * scaleFactor) + 13;
                                const scaledY1 = (y1 * scaleFactor) + 13;
                                const scaledX2 = (x2 * scaleFactor) + 13;
                                const scaledY2 = (y2 * scaleFactor) + 13;
                        
                                // 绘制线段
                                const line = two.makeLine(scaledX1, scaledY1, scaledX2, scaledY2);
                                line.stroke = 'red'; // 设置线段颜色
                                line.linewidth = 2; // 设置线段宽度
                            }
                        });
                    }
                    // 标记为已绘制
                    hasDrawn = true;
                    }
                    

                } catch (e) {
                    console.error("JSON ERROR: ", e);
                }
            } else {
                console.warn("Receive Empty Message");
            }
        }
    }
    // 如果有地图加载，重置时一并清除
    if(isMapLoaded){
        clearMapImage();
    }
    
}






/*加载地图按钮逻辑部分 */


// 更新地图图片的函数
function updateMapImage() {
    var mapSelect = document.getElementById('mapSelect');
    var mapContainer = document.getElementById('mapContainer');
    var mapImage = mapContainer.querySelector('.map-image');
    var selectedMap = mapSelect.value;

    // 设置地图图片的源
    mapImage.src = '../maps/' + selectedMap + '.png'; // 假设地图图片以.jpg格式存储
    mapContainer.style.display = 'block'; // 显示地图容器

    // 更新按钮文本为“清除地图”
    document.getElementById('mapActionButton').textContent = '清除地图';
    
    // 设置标志变量为真，表示地图已加载
    isMapLoaded = true;
}

// 清除地图的函数
function clearMapImage() {
    var mapContainer = document.getElementById('mapContainer');
    mapContainer.style.display = 'none'; // 隐藏地图容器
    mapContainer.querySelector('.map-image').src = ''; // 清除图片源

    // 更新按钮文本为“加载地图”
    document.getElementById('mapActionButton').textContent = '加载地图';
    
    // 重置标志变量，表示地图未加载
    isMapLoaded = false;
}

// 为地图操作按钮添加事件监听器
document.getElementById('mapActionButton').addEventListener('click', function() {
    if (this.textContent === '加载地图') {
        updateMapImage();
        isMapLoaded = true; // 设置标志变量为真，表示地图已显式加载
    } else {
        clearMapImage();
    }
});

// 为地图选择框添加事件监听器，仅在地图未加载时允许自动更新
document.getElementById('mapSelect').addEventListener('change', function() {
    var mapSelect = document.getElementById('mapSelect');
    
    var selectedMap = mapSelect.value;
    mapflag = parseInt(selectedMap.slice(3), 10);
});

// 初始化时隐藏地图容器
document.getElementById('mapContainer').style.display = 'none';


/*算法介绍逻辑部分 */
// 定义一个函数来更新算法介绍的文本内容
function updateAlgorithmDescription() {
    var algorithmSelect = document.getElementById('algorithmSelect');
    var algorithmDescription = document.getElementById('algorithmDescription');
    var selectedAlgorithm = algorithmSelect.value;
    var codeDescription = document.getElementById('codeDisplay');

    switch (selectedAlgorithm) {
        case 'DWA':
            algorithmDescription.innerHTML = `
            DWA算法（Dynamic Window Approach）是一种基于动态窗口的局部运动规划算法。
            它通过在给定的时间窗口内评估机器人的控制输入，选择一个能够最大化目标函数的控制输入。
            DWA算法考虑了机器人的动态特性和障碍物的存在，能够实时地规划出一条平滑且避障的路径。
            `;
            break;

        case 'A*':
            algorithmDescription.innerHTML = `
            A*算法是一种启发式搜索算法，用于在图中找到从起始点到目标点的最短路径。
            它通过评估每个节点的启发式成本（h）和实际成本（g）来选择下一步的最佳路径。
            A*算法是完备的和最优的，适用于各种路径规划问题，包括二维和三维空间。
            `;
            break;

        case 'frenet':
            algorithmDescription.innerHTML = `
            Frenet路径规划是一种基于Frenet坐标系的路径规划方法。
            它将路径规划问题分解为纵向（速度）和横向（方向）两个独立的子问题。
            Frenet路径规划特别适用于车辆和机器人的路径规划，因为它能够更好地处理动态障碍物和复杂的交通规则。
            `;
            break;

        case 'bezier':
            algorithmDescription.innerHTML = `
            Bezier路径规划是一种基于Bezier曲线的平滑路径生成方法。
            它通过定义曲线的控制点来生成一条平滑的路径，适用于需要精确控制路径形状的场景。
            Bezier路径规划可以生成复杂且平滑的路径，但可能需要更多的计算资源。
            `;
            break;

        case 'clothoid_path_planner':
            algorithmDescription.innerHTML = `
            Clothoid路径规划是一种基于clothoid曲线的路径规划方法。
            Clothoid曲线是一种具有恒定曲率变化率的曲线，适用于生成平滑且连续的路径。
            Clothoid路径规划特别适用于车辆和机器人的路径规划，因为它能够生成符合车辆动力学特性的路径。
            `;
            break;

        case 'a_star_variants':
            algorithmDescription.innerHTML = `
            A*变体算法是A*算法的一系列改进和扩展。
            这些变体包括跳点搜索、迭代深化、动态权重调整和Theta*等。
            A*变体算法旨在提高A*算法的效率和适用性，解决特定问题或优化特定性能。
            `;
            break;

        case 'bug0':
            algorithmDescription.innerHTML = `
            BUG0算法是一种基于BUG算法的路径规划方法。
            它通过在配置空间中搜索一条从起始点到目标点的路径，同时避免障碍物。
            BUG0算法适用于简单的路径规划问题，但可能在复杂环境中表现不佳。
            `;
            break;

        case 'bug1':
            algorithmDescription.innerHTML = `
            BUG1算法是BUG0算法的改进版本，它通过引入更多的启发式信息来提高搜索效率。
            BUG1算法在保持BUG0算法简单性的同时，能够更好地处理复杂环境中的路径规划问题。
            `;
            break;

        case 'bug2':
            algorithmDescription.innerHTML = `
            BUG2算法是BUG1算法的进一步改进，它通过引入更多的优化技术来提高路径规划的性能。
            BUG2算法在保持BUG1算法效率的同时，能够生成更加平滑和优化的路径。
            `;
            break;

        case 'depth_first_search':
            algorithmDescription.innerHTML = `
            深度优先搜索（Depth-First Search, DFS）是一种用于遍历或搜索树或图的算法。
            它从根节点开始，尽可能深地搜索树的分支，直到找到目标节点或到达叶子节点。
            DFS算法适用于需要全面搜索的场景，但可能不如广度优先搜索高效。
            `;
            break;

        case 'dijkstra':
            algorithmDescription.innerHTML = `
            Dijkstra算法是一种用于在加权图中找到单源最短路径的算法。
            它通过维护一个优先队列来选择下一个最近的节点，直到找到目标节点。
            Dijkstra算法是贪心算法的典型代表，适用于没有负权重边的图。
            `;
            break;

        case 'bidirectional_a_star':
            algorithmDescription.innerHTML = `
            双向A*算法是一种A*算法的变体，它从起始点和目标点同时进行搜索。
            通过同时扩展两个搜索树，双向A*算法能够更快地找到路径，特别是在大规模图中。
            这种算法适用于需要快速找到路径的场景，但计算量可能较大。
            `;
            break;

        case 'bidirectional_breadth_first_search':
            algorithmDescription.innerHTML = `
            双向广度优先搜索（Bidirectional Breadth-First Search, BBFS）是一种广度优先搜索的变体。
            它从起始点和目标点同时进行搜索，通过同时扩展两个搜索队列来加速路径的发现。
            BBFS适用于需要快速找到路径的场景，特别是在密集图中。
            `;
            break;

        case 'breadth_first_search':
            algorithmDescription.innerHTML = `
            广度优先搜索（Breadth-First Search, BFS）是一种用于遍历或搜索树或图的算法。
            它从根节点开始，逐层搜索所有节点，直到找到目标节点。
            BFS算法适用于需要找到最短路径的场景，但可能不如深度优先搜索深入。
            `;
            break;

        case 'greedy_best_first_search':
            algorithmDescription.innerHTML = `
            贪婪最佳优先搜索（Greedy Best-First Search）是一种启发式搜索算法。
            它通过选择具有最低启发式成本的节点来扩展搜索树，直到找到目标节点。
            贪婪最佳优先搜索适用于目标节点启发式成本较低的场景，但可能不是最优的。
            `;
            break;

        case 'rrt':
            algorithmDescription.innerHTML = `
            RRT算法（Rapidly-exploring Random Tree）是一种基于随机采样的路径规划算法。
            它通过在配置空间中随机撒点并连接有效点来构建搜索树，从而将连续空间的路径规划问题转化为离散空间的树搜索问题。
            RRT算法特别适用于高维空间和复杂约束条件下的路径规划，具有概率完备性，但不是最优的。
            `;
            break;

        case 'rrt_with_pathsmoothing':
            algorithmDescription.innerHTML = `
            RRT路径平滑算法是一种RRT算法的变体，它在生成路径后对路径进行平滑处理。
            通过对路径进行后处理，RRT路径平滑算法能够生成更加平滑和优化的路径。
            这种算法适用于需要生成平滑路径的场景，但计算量可能较大。
            `;
            break;

        case 'rrt_star':
            algorithmDescription.innerHTML = `
            RRT*算法（Rapidly-exploring Random Tree Star）是一种改进的RRT算法。
            它通过引入最优性条件和重新规划机制来提高路径的优化程度。
            RRT*算法在保持RRT算法概率完备性的同时，能够生成更加优化的路径。
            `;
            break;

        case 'dstar':
            algorithmDescription.innerHTML = `
            D*算法（D-Star）是一种基于动态A*的路径规划算法。
            它通过动态地更新路径来适应环境的变化，适用于动态环境中的路径规划。
            D*算法能够快速响应环境变化，但计算量较大，适用于需要实时更新路径的场景。
            `;
            break;

        case 'probabilistic_road_map':
            algorithmDescription.innerHTML = `
            概率路图（Probabilistic Roadmap, PRM）是一种基于随机采样的路径规划算法。
            它通过在配置空间中随机撒点并连接有效点来构建概率路图，从而将连续空间的路径规划问题转化为离散空间的图搜索问题。
            PRM算法特别适用于高维空间和复杂约束条件下的路径规划，具有概率完备性，但不是最优的。
            它通过较少的随机采样点即可找到一个解，随着采样点的增加，找到路径的概率逐渐趋向于1。
            `;
            break;

        case 'potential_field_planning':
            algorithmDescription.innerHTML = `
            势场规划（Potential Field Planning）是一种基于势场理论的路径规划方法。
            它通过在环境中定义吸引势场和排斥势场来引导机器人从起始点移动到目标点。
            势场规划简单直观，适用于需要快速避障的场景，但可能存在局部最小值问题。
            `;
            break;
        // 可以根据需要添加更多算法的介绍
        default:
            algorithmDescription.innerHTML = '请选择一个算法来查看介绍。';
    }
}
// 为选择框添加事件监听器，当选项变化时更新算法介绍
document.getElementById('algorithmSelect').addEventListener('change', updateAlgorithmDescription);

// 初始化算法介绍
updateAlgorithmDescription();

// 定义一个函数来更新代码展示区的内容
// function updateCodeDisplay() {
//     var xhr = new XMLHttpRequest();
//     xhr.open('GET', 'dijstra.txt', true);
//     xhr.onreadystatechange = function() {
//     if (xhr.readyState == 4 && xhr.status == 200) {
//         document.getElementById('codeDisplay').textContent = xhr.responseText;
//     }
// };
// xhr.send();
// }

/*代码展示逻辑部分 */
function updateCodeDisplay() {
    var algorithmSelect = document.getElementById('algorithmSelect');
    var codeDisplay = document.getElementById('codeDisplay');
    var selectedAlgorithm = algorithmSelect.value;

    switch (selectedAlgorithm) {
        case 'Dijkstra':
            codeDisplay.textContent = `def dijkstra(graph, start):
    # 初始化距离字典，所有距离设为无穷大，除了起点
    distances = {vertex: float('infinity') for vertex in graph}
    distances[start] = 0
    # 优先队列，存储（距离，顶点）对
    priority_queue = [(0, start)]
    while priority_queue:
        # 取出当前距离最小的顶点
        current_distance, current_vertex = heapq.heappop(priority_queue)
        # 如果取出的距离大于已记录的距离，则跳过此次循环
        if current_distance > distances[current_vertex]:
            continue
        # 遍历当前顶点的所有邻居
        for neighbor, weight in graph[current_vertex].items():
            distance = current_distance + weight
            # 只有找到更短的路径时才更新
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
    return distances
            `
            break;
        case 'A*':
            codeDisplay.textContent = `def heuristic(a, b):
    """使用曼哈顿距离作为启发式函数"""
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def a_star_search(start, goal, graph):
    """A*算法实现"""
    # 开放列表，用于存储待处理的节点，元素格式为(成本, 节点)
    open_list = []
    # 将起点加入开放列表
    heapq.heappush(open_list, (0, start))
    
    # 关闭列表，用于存储已经处理过的节点
    closed_list = set()
    
    # 记录每个节点的父节点，用于最后重建路径
    came_from = {}
    
    # 记录每个节点到达的最低成本
    cost_so_far = {start: 0}
    
    while open_list:
        # 从开放列表中取出成本最低的节点
        _, current = heapq.heappop(open_list)
        
        # 如果到达目标节点，重建并返回路径
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        # 将当前节点加入关闭列表
        closed_list.add(current)
        
        # 遍历邻居节点
        for neighbor, weight in graph.get(current, {}).items():
            # 如果邻居不在关闭列表中，并且能到达
            if neighbor not in closed_list and weight is not None:
                new_cost = cost_so_far[current] + weight
                # 如果找到更低的成本或邻居不在开放列表中
                if (neighbor not in cost_so_far or 
                    new_cost < cost_so_far[neighbor]):
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (priority, neighbor))
                    came_from[neighbor] = current
    
    # 如果没有路径到达目标
    return None`;
            break;
        case 'PRM':
            codeDisplay.textContent = `def generate_random_points(num_points, x_bounds, y_bounds):
    """在给定的边界内生成随机点"""
    points = np.random.uniform(low=[x_bounds[0], y_bounds[0]], high=[x_bounds[1], y_bounds[1]], size=(num_points, 2))
    return points

def create_roadmap(points, distance_threshold):
    """根据距离阈值连接点"""
    roadmap = []
    for i, point1 in enumerate(points):
        for j, point2 in enumerate(points[i+1:], start=i+1):
            distance = np.linalg.norm(point1 - point2)
            if distance <= distance_threshold:
                roadmap.append((i, j, distance))
    return roadmap

defprm_planning(start, goal, roadmap):
    """在PRM图上搜索路径"""
    # 使用图搜索算法找到路径
    # 这里省略了图搜索的具体实现
    path = []
    return path
`;
            break;
        case 'RRT':
            codeDisplay.textContent = `class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class RRT:
    def __init__(self, start, goal, max_iter=1000, step_size=1.0, obstacle_list=[]):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.max_iter = max_iter
        self.step_size = step_size
        self.obstacle_list = obstacle_list
        self.tree = [self.start]

    def is_collision(self, node1, node2):
        for obs in self.obstacle_list:
            if self.check_collision(node1, node2, obs):
                return True
        return False

    def check_collision(self, node1, node2, obs):
        # 这里简化处理，只考虑圆形障碍物
        dist = np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
        return dist < obs[2] + 0.1  # 0.1是安全距离

    def steer(self, from_node, to_point):
        direction = np.array([to_point[0] - from_node.x, to_point[1] - from_node.y])
        new_point = from_node.x + direction * min(1, self.step_size / np.linalg.norm(direction))
        return Node(new_point[0], new_point[1])

    def grow_tree(self):
        for i in range(self.max_iter):
            rand_point = np.random.uniform(-10, 10, 2)  # 随机点
            nearest_node = self.get_nearest_node(rand_point)
            if not self.is_collision(nearest_node, rand_point):
                new_node = self.steer(nearest_node, rand_point)
                if not self.is_collision(new_node, nearest_node):
                    self.tree.append(new_node)
                    if np.linalg.norm(np.array([new_node.x, new_node.y]) - np.array([self.goal.x, self.goal.y])) < 1:
                        return self.reconstruct_path(new_node)
        return None

    def get_nearest_node(self, point):
        nearest_node = None
        min_dist = float('inf')
        for node in self.tree:
            dist = np.linalg.norm(np.array([node.x, node.y]) - np.array([point[0], point[1]]))
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def reconstruct_path(self, node):
        path = []
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]

    def plot_tree(self, path):
        plt.clf()
        if path:
            for i in range(len(path)-1):
                plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], 'k-')
        plt.plot(self.start.x, self.start.y, 'go')
        plt.plot(self.goal.x, self.goal.y, 'ro')
        for obs in self.obstacle_list:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='k')
            plt.gca().add_patch(circle)
        plt.xlim(-15, 15)
        plt.ylim(-15, 15)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.draw()
        plt.pause(0.01)
`;
            break;
        // 可以根据需要添加更多算法的代码
        default:
            codeDisplay.textContent = '# 请选择一个算法来查看示例代码';
    }
}

// 为算法选择框添加事件监听器
document.getElementById('algorithmSelect').addEventListener('change', updateCodeDisplay);

// 初始化代码展示区
updateCodeDisplay();


/*地图坐标点击选择逻辑部分 */
const mapContainer = document.getElementById('mapContainer');
var startCoordInput = document.getElementById('startCoord');
var endCoordInput = document.getElementById('endCoord');

//为地图容器添加点击事件监听器
mapContainer.addEventListener('click', function(event) {
    const rect = clickLayer.getBoundingClientRect();
    // const x = event.clientX - rect.left *(10/729); // 获取点击的X坐标
    // const y = (event.clientY - rect.top) *(10/540);  // 获取点击的Y坐标
    const x = parseFloat(((event.clientX - rect.left) *(27/729)).toFixed(2));;
    const y = parseFloat(((event.clientY - rect.top) * (20/540)).toFixed(2)); 
    if (activeInput) {
        activeInput.value = `${x},${y}`;
    }
});
const clickLayer = document.getElementById('clickLayer');
let activeInput = null; // 当前激活的输入框

// 为起点和终点输入框添加点击事件监听器
startCoordInput.addEventListener('click', function() {
    activeInput = startCoordInput;
});

endCoordInput.addEventListener('click', function() {
    activeInput = endCoordInput;
});

// // 为透明层添加点击事件监听器
// clickLayer.addEventListener('click', function(event) {
//     const rect = clickLayer.getBoundingClientRect();
//     const x = event.clientX - rect.left;
//     const y = event.clientY - rect.top;
//     console.log(activeInput==null);
//     if (activeInput) {
//         activeInput.value = `${x},${y}`;
//     }
// });