/*MQTT通信，数据收发及路径绘制部分 */
let isRunning = false;// 控制运行状态的标志
function runExperiment() {
    const container = document.getElementById('animation-container');
    container.innerHTML = ' ';
    //var selectedAlgorithm = document.getElementById('algorithmSelect').value;
    var startCoord = document.getElementById('startCoord').value.split(',').map(Number);
    console.log("起点: ", startCoord);
    var endCoord = document.getElementById('endCoord').value.split(',').map(Number);
    //var resultDetails = document.getElementById('resultDetails');
    //var experimentCanvas = document.getElementById('experimentCanvas');
    var runButton = document.getElementById('runButton');
    if(isRunning){
       //resultDetails.innerHTML = '结果将显示在这里...';
        runButton.textContent = '运行实验';
        isRunning = false;
        return;
    }
    isRunning = true;
    runButton.textContent = '重置实验';
    	
    // 创建 MQTT 客户端
    const client = new Paho.Client("10.180.101.139", 8083, "clientId-" + Math.random());

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
            '目标点': endCoord
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

    let hasDrawn = false;

    // 消息到达时的回调
    function onMessageArrived(message) {

        if (hasDrawn) return;

        console.log("收到的完整消息: ", message.payloadString);
        if (message.payloadString) {
            try {
                const data = JSON.parse(message.payloadString);
                console.log("解析后的数据: ", data);
                
                // 解析 JSON 数据
                const { 起点, 目标点, 障碍物, 车辆行使路径 } = data;

                // 初始化 Two.js

                const elem = document.getElementById('animation-container');


                const params = { width: 800, height: 600 };
                const two = new Two(params).appendTo(elem);
                two.clear();
                
                // 缩放因子
                const scaleFactor = 30;
               

                // 绘制起点
                const startCircle = two.makeCircle(startCoord[0] * scaleFactor+15, startCoord[1] * scaleFactor+15, 15);
                startCircle.fill = 'blue';

                // 绘制目标点
                const targetCircle = two.makeCircle(endCoord [0] * scaleFactor+15, endCoord [1] * scaleFactor+15, 15);
                targetCircle.fill = 'green';

                // 绘制障碍物
                障碍物.forEach(([x, y]) => {
                    const obstacleCircle = two.makeRectangle(x * scaleFactor+15, y * scaleFactor+15, 30, 30);
                    obstacleCircle.fill = 'red';
                });

                // 动态绘制车辆路径
                let currentIndex = 0;
                const vehicle = two.makeCircle(车辆行使路径[0][0] * scaleFactor+15, 车辆行使路径[0][1] * scaleFactor+15, 15);
                //const vehicle = two.makeRectangle(车辆行使路径[0][0] * scaleFactor+15, 车辆行使路径[0][1] * scaleFactor+15, 20,30);
                vehicle.fill = 'yellow';

                two.bind('update', function (frameCount) {
                    if (currentIndex < 车辆行使路径.length - 1) {
                        const [x1, y1] = 车辆行使路径[currentIndex];
                        const [x2, y2] = 车辆行使路径[currentIndex + 1];
                        vehicle.translation.set(x2 * scaleFactor+15, y2 * scaleFactor+15);

                        // 绘制线段
                        const line = two.makeLine(x1 * scaleFactor+15, y1 * scaleFactor+15, x2 * scaleFactor+15, y2 * scaleFactor+15);
                        line.stroke = 'black';
                        line.linewidth = 2;

                        currentIndex++;
                    }
                }).play();

                // 标记为已绘制
                hasDrawn = true;

            } catch (e) {
                console.error("JSON ERROR: ", e);
            }
        } else {
            console.warn("Receive Empty Message");
        }
    }
    
}






/*加载地图按钮逻辑部分 */
// 标志变量，初始时不允许自动加载地图
let isMapLoaded = false;

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

// // 为地图选择框添加事件监听器，仅在地图未加载时允许自动更新
// document.getElementById('mapSelect').addEventListener('change', function() {
//     if (!isMapLoaded) {
//         // 如果地图未加载，则不自动更新地图
//         //this.value = ''; // 清除选择框的选项
//         //alert('请选择地图后，点击“加载地图”按钮来加载地图。');
//     }
// });

// 初始化时隐藏地图容器
document.getElementById('mapContainer').style.display = 'none';


/*算法介绍逻辑部分 */
// 定义一个函数来更新算法介绍的文本内容
function updateAlgorithmDescription() {
    var algorithmSelect = document.getElementById('algorithmSelect');
    var algorithmDescription = document.getElementById('algorithmDescription');
    var selectedAlgorithm = algorithmSelect.value;

    switch (selectedAlgorithm) {
        case 'DWA':
            algorithmDescription.innerHTML = `动态窗口法（Dynamic Window Approach，简称DWA）是一种局部路径规划算法，主要用于移动机器人在动态环境中的避障和路径规划。
            DWA算法的核心思想是在速度空间中采样一系列可能的速度值，然后预测每个速度值在一定时间内的轨迹，并使用评价函数来评估这些轨迹的优劣，最终选择最优的轨迹作为机器人的控制指令。
            `;
            break;
            
        case 'Teb':
            algorithmDescription.innerHTML = `
            TEB（Time Elastic Band）算法是一种局部路径规划方法，主要用于移动机器人在动态环境中的避障和路径优化。
            它通过时间弹性带的概念来表示机器人的轨迹，允许机器人在保持连续性和平滑性的同时，动态地调整运动轨迹以避开障碍物。
            TEB算法利用优化框架搜索最优轨迹，通常采用非线性优化方法，如g2o库，来最小化轨迹的成本函数。
            该算法考虑机器人的动态特性，通过动态窗口法选择可行的速度和方向，适用于实时路径规划。
            TEB算法提供了多种参数来调整规划行为，如最大速度、加速度、转向半径等，可根据具体应用场景进行调整。
            它常与全局路径规划算法结合使用，以提供全局和局部的路径规划解决方案，适用于需要考虑机器人动态特性和实时响应的环境中。
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