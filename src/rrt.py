"""
version1.1,2018-05-09
《基于智能优化与RRT算法的无人机任务规划方法研究》博士论文
《基于改进人工势场法的路径规划算法研究》硕士论文
reference:
https://www.cnblogs.com/yangmingustb/p/9013534.html?ivk_sa=1024320u

"""

import matplotlib.pyplot as plt
import random
import math
import copy
import numpy as np

show_animation = True


class Node(object):
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT(object):
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacle_list, rand_area):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:random sampling Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        # self.expandDis = 1.0
        self.expandDis = 0.2
        self.goalSampleRate = 0.05  # 选择终点的概率是0.05
        self.maxIter = 500
        self.obstacleList = obstacle_list
        self.nodeList = [self.start]

    def random_node(self):
        """
        产生随机节点
        :return:
        """
        # import ipdb;ipdb.set_trace()
        node_x = random.uniform(self.min_rand, self.max_rand)
        node_y = random.uniform(self.min_rand, self.max_rand)
        node = [node_x, node_y]

        return node

    @staticmethod
    def get_nearest_list_index(node_list, rnd):
        """
        :param node_list:
        :param rnd:
        :return:
        """
        # import ipdb;ipdb.set_trace()
        d_list = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list]
        min_index = d_list.index(min(d_list))
        return min_index

    @staticmethod
    def collision_check(new_node, obstacle_list):
        # import ipdb;ipdb.set_trace()
        a = 1
        for (ox, oy, size) in obstacle_list:
            dx = ox - new_node.x
            dy = oy - new_node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                a = 0  # collision

        return a  # safe

    def planning(self):
        """
        Path planning

        animation: flag for animation on or off
        """

        while True:
            # import ipdb;ipdb.set_trace()
            # Random Sampling
            if random.random() > self.goalSampleRate:
                rnd = self.random_node()
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            min_index = self.get_nearest_list_index(self.nodeList, rnd)
            # print(min_index)

            # expand tree
            nearest_node = self.nodeList[min_index]

            # 返回弧度制
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)

            new_node = copy.deepcopy(nearest_node)
            new_node.x += self.expandDis * math.cos(theta)
            new_node.y += self.expandDis * math.sin(theta)
            new_node.parent = min_index
            
            # print(f'current new node: {new_node.x}, {new_node.y}')
            
            if not self.collision_check(new_node, self.obstacleList):
                continue
            
            # print(f'current new node: {new_node.x}, {new_node.y}')
            self.nodeList.append(new_node)

            # check goal
            dx = new_node.x - self.end.x
            dy = new_node.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if True:
                self.draw_graph(rnd)

        path = [[self.end.x, self.end.y]]
        last_index = len(self.nodeList) - 1
        while self.nodeList[last_index].parent is not None:
            node = self.nodeList[last_index]
            path.append([node.x, node.y])
            last_index = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def draw_graph(self, rnd=None):
        """
        Draw Graph
        """
        print('aaa')
        plt.clf()  # 清除上次画的图
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^g")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                        node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "sk", ms=10*(2*size)**2)

        plt.plot(self.start.x, self.start.y, "^r")
        plt.plot(self.end.x, self.end.y, "^b")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)
        # import ipdb;ipdb.set_trace()

    def draw_static(self, path):
        """
        画出静态图像
        import ipdb;ipdb.set_trace()
        :return:
        """
        plt.clf()  # 清除上次画的图

        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "sk", ms=20*size)

        plt.plot(self.start.x, self.start.y, "^r")
        plt.plot(self.end.x, self.end.y, "^b")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])

        plt.plot([data[0] for data in path], [data[1] for data in path], '-r')
        plt.grid(True)
        plt.show()


def main():
    print("start RRT path planning")

    obstacle_list = [
        (5, 1, np.sqrt(2)/2),
        (3, 6, np.sqrt(2)/2),
        (3, 8, np.sqrt(2)/2),
        (1, 1, np.sqrt(2)/2),
        (3, 5, np.sqrt(2)/2),
        (9, 5, np.sqrt(2)/2)]

    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[8, 9], rand_area=[-2, 10], obstacle_list=obstacle_list)
    path = rrt.planning()
    print(path)

    # Draw final path
    if show_animation:
        plt.close()
        rrt.draw_static(path)


if __name__ == '__main__':
    main() 
