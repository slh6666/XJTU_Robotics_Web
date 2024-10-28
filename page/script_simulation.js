/*MQTT通信，数据收发及路径绘制部分 */
let isRunning = false;// 控制运行状态的标志
let algorithmflag = '';//后端通信的算法选择标志位
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

            // 将数组打包为 JSON 对象
            var dataToSend = {
                '起点': startCoord,
                '目标点': endCoord,
                'code': selectedAlgorithm,
                'map': mapflag
            };

            // 将 JSON 对象转换为字符串
            const jsonString = JSON.stringify(dataToSend);

            // 发送 JSON 数据到指定主题
            const topic = "/clientinfo";
            const localmsg = new Paho.Message(jsonString);
            localmsg.destinationName = topic;
            client.send(localmsg);
            console.log("Local Message Send!: ", jsonString);
            client.subscribe("/planning003");
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
                    const { 起点, 目标点, 障碍物, 车辆行使路径 , path} = data;
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
        case 'Dijkstra':
            algorithmDescription.innerHTML = 'Dijkstra算法是一种图搜索算法，由荷兰计算机科学家艾兹赫尔·Dijkstra在1956年提出。'+
            '它用于寻找图中两点间的最短路径。算法从起点开始，逐步扩展到邻近节点，更新路径长度，直到到达目标点或所有可达节点都被访问。'+
            'Dijkstra算法适用于有向图和无向图，且边权重为非负数的情况。它在网络路由、地图导航等领域有广泛应用。由于其高效性，Dijkstra算法是解决最短路径问题的最基本和最知名的算法之一。';
            algorithmflag = 1;
            break;
        case 'A*':
            algorithmDescription.innerHTML = `
            A*算法是一种启发式搜索算法，用于在图中找到从起始节点到目标节点的最短路径。
            它结合了Dijkstra算法和最佳优先搜索的特点，使用启发式函数来估计从当前节点到目标节点的成本。
            A*算法在每一步都选择看似最优的路径，因此它在路径规划和图搜索问题中非常有效，特别是在有明确目标和启发式信息可用的情况下。
            `;
            algorithmflag = 0;
            break;
        case 'PRM':
            algorithmDescription.innerHTML = `
            PRM算法（Probabilistic Roadmap）是一种基于随机采样的路径规划算法。
            它通过在配置空间中随机撒点并连接有效点来构建概率路图，从而将连续空间的路径规划问题转化为离散空间的图搜索问题。
            PRM算法特别适用于高维空间和复杂约束条件下的路径规划，具有概率完备性，但不是最优的。
            它通过较少的随机采样点即可找到一个解，随着采样点的增加，找到路径的概率逐渐趋向于1。
            `;
            break;
        case 'RRT':
            algorithmDescription.innerHTML = `
            RRT算法（Rapidly-exploring Random Tree）是一种用于解决非结构化环境中的路径规划问题的算法。
            它通过从起点开始，随机选择一个方向和距离进行扩展，逐步构建一棵树状结构，以探索配置空间。
            RRT算法能够处理复杂的障碍物环境，并能快速找到一个可行的路径，尽管这个路径可能不是最优的。
            该算法特别适合于动态环境和需要快速响应的场景，如机器人导航和自动驾驶。
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