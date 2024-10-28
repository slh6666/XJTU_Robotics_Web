def dijkstra(graph, start):
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