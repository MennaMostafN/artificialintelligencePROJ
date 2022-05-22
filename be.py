import collections
from queue import Queue


class SearchAlgo:
    Graph = {
        # 's': [('a', 4), ('b', 8)],
        # 'a': [('c', 10)],
        # 'c': [('d', 10)],

    }

    Heuristic = {
        # 's': 1,
        # 'a': 2,
        # 'b': 1,
        # 'c': 1,
        # 'd': 0
    }
    StartNode = ""
    GoalNode = []

    def path_cost(self, path):
        total_cost = 0
        for (node, cost) in path:
            total_cost = total_cost + cost
        return total_cost, path[-1][0]

    # //////////////////////////////////UNIFORM COST////////////////////////////////////////////////////
    def UCS(self, graph, start, goal):
        visited = []
        queue = [[(start, 0)]]
        cost = 0
        while queue:
            queue.sort(key=self.path_cost)
            print(queue)
            path = queue.pop(0)
            print(path)
            node = path[-1][0]
            print(node)
            print(visited)
            visited.append(node)
            print(visited)
            if node == goal[0]:
                printed_path = []
                for (i, j) in path:
                    printed_path.append(i)

                print(printed_path)
                print(visited)

                return printed_path, visited
            else:
                adjacent_node = graph.get(node, [])
                print(adjacent_node)
                for (node2, cost) in adjacent_node:
                    print(path)
                    newpath = path.copy()
                    print(newpath)
                    newpath.append((node2, cost))
                    print(newpath)
                    queue.append(newpath)
                    print(queue)

    # ///////////////////////////////////////DEPTHLIMITED////////////////////////////////////////////////////////////
    def DLS(self, graph, start, goal, maxDepth):
        stack = [start]
        visited = []
        parent = {}
        path = []
        level = 0
        print(graph.keys())
        for node in graph.keys():
            parent[node] = None
        while stack:
            if (level > maxDepth):
                return visited, path,level
            print(stack)
            node = stack.pop(0)
            visited.append(node)
            print(visited)
            print(stack)
            print(node)
            if node == goal[0]:
                p = parent[goal[0]]
                path.append(goal[0])
                while p != None:
                    print(p)
                    path.append(p)
                    p = parent[p]
                path.reverse()
                print(parent)
                print(path)
                print(visited)
                return visited, path
            children = graph[node]
            print(children)
            for child in children:
                print(child)
                print(child[0])
                if child[0] not in visited:
                    print(node)
                    parent[child[0]] = node
                    stack.insert(0, child[0])
                    print(stack)
            if (stack[0], 0) in children:
                level = level + 1
                print(level)

    # //////////////////////////////////////////ITERATIVE////////////////////////////////////////////////////////////////////////
    def iterative(self, graph, start, goal):
        limit = 0
        while True:
            visited, path = self.DLS(graph,start,goal,limit)
            if len(path) == 0:
                limit+=1
                print(limit)
            else:
                return path, visited

    # ////////////////////////////////////////////DEPTH/////////////////////////////////////////////////////////////////
    def DFS(self, graph, start, goal):
        stack = [start]
        visited = []
        parent = {}
        path = []
        print(graph.keys())
        for node in graph.keys():
            parent[node] = None
        while stack:
            print(stack)
            node = stack.pop(0)
            visited.append(node)
            print(visited)
            print(stack)
            print(node)
            if node == goal[0]:
                p = parent[goal[0]]
                path.append(goal[0])
                while p != None:
                    print(p)
                    path.append(p)
                    p = parent[p]
                path.reverse()
                print(parent)
                print(path)
                print(visited)
                return visited, path
            children = graph[node]
            print(children)

            for child in children:
                print(child)
                if child[0] not in visited:
                    print(node)
                    parent[child[0]] = node
                    stack.insert(0, child[0])
                    print(stack)

    # /////////////////////////////////////////BREADTH//////////////////////////////////////////////////////
    def BFS(self, graph, start, goal):
        visited = []
        queue = [[start]]
        while queue:
            path = queue.pop(0)
            print(path)
            node = path[-1]
            print(node)
            print(visited)
            visited.append(node)
            print(visited)
            if node == goal[0]:
                print(path)
                print(visited)
                return path, visited
            else:
                adjacent_node = graph.get(node, [])
                print(adjacent_node)
                for node2 in adjacent_node:
                    print(path)
                    newpath = path.copy()
                    print(newpath)
                    newpath.append(node2[0])
                    print(newpath)
                    queue.append(newpath)
                    print(queue)

    # //////////////////////////////////GREEDYY/////////////////////////////////////////////////////////////////////////
    def Greedy_Search(self, graph, start, goal):
        visited = []
        queue = [start]
        parent = {}
        path = []
        for node in graph.keys():
            parent[node] = None
        print(self.Heuristic)
        print(visited)
        while queue:
            node = queue.pop(0)
            print(node[0])
            visited.append(node[0])
            print(visited)
            if node[0] == goal[0]:
                p = parent[goal[0]]
                path.append(goal[0])
                while p != None:
                    print(p)
                    path.append(p)
                    p = parent[p]
                path.reverse()
                print(parent)
                print(path)
                print(visited)
                return visited, path
            else:
                children = graph[node[0]]
                print(children)
                for child in children:
                    queue.append((child[0], self.Heuristic[child[0]]))
                    print(queue)
                    length = len(queue)
                    print(length)
                    parent[child[0]] = node[0]
                    for l in range(0, length - 1):
                        print("b")
                        if (queue[l][1] > queue[l + 1][1]):
                            tuple = queue[l]
                            queue[l] = queue[l + 1]
                            queue[l + 1] = tuple
                print(queue)
        return

    def path_h_cost(self, path):
        total_cost = 0
        for (node, cost) in path:
            total_cost = total_cost + cost + self.Heuristic[node]
        return total_cost, path[-1][0]

    # ///////////////////////////////////ASTAR//////////////////////////////////////////////////////////////////////
    def astarsearch(self, graph, start, goal):
        visited = []
        queue = [[(start, 0)]]
        cost = 0
        while queue:
            queue.sort(key=self.path_h_cost)
            print(queue)
            path = queue.pop(0)
            print(path)
            node = path[-1][0]
            print(node)
            print(visited)
            visited.append(node)
            print(visited)
            if node == goal[0]:
                printed_path = []
                for (i, j) in path:
                    printed_path.append(i)

                print(printed_path)
                print(visited)
                return printed_path, visited
            else:
                adjacent_node = graph.get(node, [])
                print(adjacent_node)
                for (node2, cost) in adjacent_node:
                    print(path)
                    newpath = path.copy()
                    print(newpath)
                    newpath.append((node2, cost))
                    print(newpath)
                    queue.append(newpath)
                    print(queue)

