import typing
import math
import dataclasses
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


@dataclasses.dataclass
class Point:
    """
    Класс для описания точек
    """
    x: float
    y: float


class Node:
    F_VALUE = 1

    def __init__(self, center_point: Point):
        """
        Класс для описания узлов графа
        @param center_point: Координаты центра узла
        """
        self.center_point: Point = center_point
        self.neighboursDir: typing.List[Node] = []
        self.neighboursDiag: typing.List[Node] = []

        self.isDeadEnd: bool = False
        self.link_to_last_node: (Node, None) = None

        self.f_value = 0
        self.f = 0
        self.path = 0
        self.mass = 0

    def add_neighbourDir(self, neighbour):
        neighbour: Node = neighbour
        if neighbour in self.neighboursDir:
            return

        self.neighboursDir.append(neighbour)

        if self in neighbour.neighboursDir:
            return
        neighbour.neighboursDir.append(self)

    def add_neighbourDiag(self, neighbour):
        neighbour: Node = neighbour
        if neighbour in self.neighboursDiag:
            return

        self.neighboursDiag.append(neighbour)

        if self in neighbour.neighboursDiag:
            return
        neighbour.neighboursDiag.append(self)

    def print_data(self):
        print(self.center_point.x, self.center_point.y, self.f, self.path, self.mass)


class Map:
    def __init__(self):
        self.nods: typing.List[typing.List[Node]] = []

        self.open_nodes: typing.List[Node] = []  # Вершины, в которых уже были
        self.explored_nodes: typing.List[Node] = []  # Исследованные вершины

        self.__sizeNode = 1

    def create_map(self, num_node: object, height: object, weight: object) -> object:
        """
        Создание карты из элементов Node
        @param num_node: кол-во узлов, на которе разбивается сторона
        @param height: ширина карты в метрах
        @param weight: высота карты в метрах
        @return:
        """
        height_one_node = height / num_node
        weight_one_node = weight / num_node

        centre_x = weight_one_node / 2
        centre_y = height_one_node / 2

        for i in range(num_node):
            nods = []
            for j in range(num_node):
                x = centre_x + i * weight_one_node - 4
                y = centre_y + j * height_one_node - 4
                centre_node = Point(x=x, y=y)
                nods.append(Node(centre_node))
            self.nods.append(nods)

        print(x, y)

        for i in range(1, num_node - 1):
            for j in range(1, num_node - 1):
                self.nods[i][j].add_neighbourDir(self.nods[i + 1][j])
                self.nods[i][j].add_neighbourDir(self.nods[i - 1][j])

                self.nods[i][j].add_neighbourDir(self.nods[i][j + 1])
                self.nods[i][j].add_neighbourDir(self.nods[i][j - 1])

                self.nods[i][j].add_neighbourDiag(self.nods[i - 1][j - 1])
                self.nods[i][j].add_neighbourDiag(self.nods[i - 1][j + 1])
                self.nods[i][j].add_neighbourDiag(self.nods[i + 1][j - 1])
                self.nods[i][j].add_neighbourDiag(self.nods[i + 1][j + 1])

    def add_block(self, point: Point):
        """
        Добавление на карту препятствий. Узел, которому принадлежит точка point будет помечена, как непроходимая
        @param point: Центр препятствия
        @return:
        """
        block_node = self.__point_to_node(point)
        block_node.isDeadEnd = True

    def __point_to_node(self, point: Point):
        for i in range(len(self.nods)):
            for j in range(len(self.nods[i])):
                res = self.__check_distance_in_square(point, self.nods[i][j].center_point, 0.5)
                if res:
                    return self.nods[i][j]

    def __calculation_heuristic(self, point1: Point, point2: Point):
        return math.dist((point1.x, point1.y), (point2.x, point2.y)) * 2

    def get_trajectory(self, start_point: Point, end_point: Point):
        """ Полчение траектории """
        start_node = self.__point_to_node(start_point)
        end_node = self.__point_to_node(end_point)

        if end_node.isDeadEnd:
            print('Конечная точка недостигаема')
            return

        is_end_node = False

        cur_node = start_node
        while True:
            # print("_____________________________________________________________")
            # print("cur_node ", (cur_node.center_point.x, cur_node.center_point.y))

            neighboursCurNode = cur_node.neighboursDir + cur_node.neighboursDiag
            for node in neighboursCurNode:
                F = 1 if (node in cur_node.neighboursDir) else 1.5

                if node.isDeadEnd:
                    continue

                if node in self.explored_nodes:
                    continue

                if node not in self.open_nodes:
                    self.open_nodes.append(node)

                if (node.f == 0) or (cur_node.f + F < node.f):
                    node.f += F + cur_node.f
                    node.path = self.__calculation_heuristic(end_node.center_point, node.center_point)
                    node.mass = node.f + node.path
                    node.link_to_last_node = cur_node
                else:
                    self.explored_nodes.append(node)

                # node.print_data()
                if node == end_node:
                    is_end_node = True
                    break

            if is_end_node:
                break

            next_node = self.open_nodes[0]
            for node in self.open_nodes:
                if node.mass <= next_node.mass:
                    next_node = node

            self.open_nodes.remove(next_node)

            self.explored_nodes.append(cur_node)
            cur_node = next_node

        trajectory_node = []
        node = end_node
        while True:
            trajectory_node.append(node)
            if node == start_node:
                break
            node = node.link_to_last_node

        i = 0
        while True:
            if (i >= len(trajectory_node) - 1) or (i + 1 >= len(trajectory_node)) or (i + 2 >= len(trajectory_node)):
                break
            res_check = self.__check_in_line(trajectory_node[i].center_point,
                                             trajectory_node[i + 1].center_point,
                                             trajectory_node[i + 2].center_point)
            if res_check:
                trajectory_node.remove(trajectory_node[i + 1])
                print("Точка удалена")
            else:
                i = i + 1
        new_tr = []
        for t in trajectory_node:
            new_tr.append(t.center_point)

        return new_tr

    @classmethod
    def __check_in_line(cls, point1: Point, point2: Point, point3: Point):
        if (point2.x - point1.x) * (point3.y - point1.y) - (point3.x - point1.x) * (point2.y - point1.y):

            return False
        else:
            print("Точки: ", point1, point2, point3, " лежат на одной прямой")
            return True

    @staticmethod
    def __check_distance_in_square(point1: Point, point2: Point, dist_to_collision: float) -> bool:
        """
            Проверка нахождения точки point1 внутри квадрата с центром в point2 и стороной dist_to_collision * 2
            :param point1:
            :param point2:
            :return:
        """
        if ((point2.x + dist_to_collision) > point1.x > (point2.x - dist_to_collision)) and (
                (point2.y + dist_to_collision) > point1.y > (point2.y - dist_to_collision)):
            return True
        else:
            return False

def write_on_plt(m, tr):
    x = []
    y = []
    for t in tr:
        x.append(t.x)
        y.append(t.y)

    fig, ax = plt.subplots(figsize=(6, 6))

    ax.set_xlim([-4, 4])
    ax.set_ylim([-4, 4])
    ax.grid()

    sizeNode = 0.25
    for nodeColumn in m.nods:
        for node in nodeColumn:
            pos = node.center_point
            ax.scatter(x=pos.x, y=pos.y, marker='o', c='r', edgecolor='b')
            color = "red" if node.isDeadEnd else "white"
            ax.add_patch(Rectangle((pos.x - sizeNode / 2, pos.y - sizeNode / 2), sizeNode, sizeNode, facecolor=color,
                                   edgecolor='black'))

    for node in m.explored_nodes:
        pos = node.center_point
        ax.scatter(x=pos.x, y=pos.y, marker='o', c='r', edgecolor='b')
        color = "grey"
        ax.add_patch(Rectangle((pos.x - sizeNode / 2, pos.y - sizeNode / 2), sizeNode, sizeNode, facecolor=color,
                               edgecolor='black'))

    for i, t in enumerate(tr):
        color = "green" if i == 0 or i == (len(tr) - 1) else "blue"
        ax.scatter(x=t.x, y=t.y, marker='o', c='r', edgecolor='b')
        ax.add_patch(Rectangle((t.x - sizeNode / 2, t.y - sizeNode / 2), sizeNode, sizeNode, facecolor=color,
                               edgecolor='black'))

    plt.plot(x, y)
    plt.show()


if __name__ == "__main__":
    MAP_BLOCK_LIST=[
        (-2.25, -1.5),
        (-2.04, -1.63),
        (-2.09, -1.37),
        (-1.36, -0.47),
        (-1.3, -1.11),
        (-1.7, -0.42),
        (-1.45, -0.54),
        (-1.64, -0.81),

        (-3.11, 2.69),
        (-2.12, 4.03),
        (-1.27, 1.62),
        (-2.29, 2.01),
        (-2.29, 1.54),
        (-3.76, 0.76),
        (-1.36, -0.78),
        (-2.65, -0.17),
        (-3.25, -1.96),
        (-2.11, -1.49),
        (-2.23, -3.71),
        (-0.73, -2.99),
        (-0.7, -3.29),
        #(-0.2, -3.1),

        (-0.64,-3.91),
        (-0.115, -3.027),
        (-0.31, -1.91),
        (-0.97, 0.46),
        (0.5, 2.66),
        (-0.1, 3.07),
        (-0.11, 2.74),
        (1.78, 4.11),
        (2.39, 3.8),
        (1.88, 3.83),
        (2.42, 4.2),
        (1.9, 2.05),
        (3.03, 2.01),
        (4.22, 1.9),
        (3.89, 2.15),
        (2.64, 0.71),
        (1.84, 0.388),
        (1.94, 0.061),
        (2.58, -0.25),
        (2.06, -0.266),
        (2.56, 0.09),
        (-0.12, -0.33),
        (0.48, -0.31),
        (0.18, 0.01),
        (-0.32, -0.92),
        (1.62, -2.38),
        (2.13, -2.38),
        (1.88, -2.61),
        (1.83, -2.14),
        (0.81, -3),
        (1.32, -3.32),
        (1.28, -3.04),
        (4.02, -0.88),
        (4, -1.24),
        (4, -1.27),
        (4, -0.88),
        (2.9, -1.43),
        (0.49, -1.86),
        (1.31, -0.95),
        (0.85, 0.24),
        (-0.038, 1.49),

        (-0.5, -2),
        (0.31, 3),
        (0.08, 2.8),
        (0.19, -0.17),
        (-1.12, -0.94),
        (-0.6, -1.76),
        (-1.12, -0.64),
        (-1.6, -1),
        (2.31, -0.008),
        (2.31, -0.33),
        (2.08, 4.03),
        (2.11, 3.75),
        (3.99, 2.04),
        (-3.61, 0.84),
        (-3.52, 0.56),
        (-3.54, 0.93),



    ]
    MAP_BLOCK_LIST_ = [
        # зеленые домики
        (-0.9, -4),
        (-0.9, -3),
        (4, -1),
        (-3.2, 2.7),
        (-2.2, 4.1),
        (0.2, 2.9),
        (-0.1, 3.2),
        (-0.1, 2.7),
        (0.5, 2.7),
        (0.5, 2.9),

        # красные домики
        (-0.77, -2),
        (-0.77, -2),
        (-3.3, -1.86),
        (-1.37, -0.76),
        (-1.19, -0.94),
        (0.2, 2.9),

        # синие домики
        (1.2, -3.1),
        (1.8, -2.3),
        (-3.8, 0.7),

        # cерые домики
        (0, 0),
        (0, -0.29),
        (0.3, -0.29),

        (-2.2, 4),
        (2, 0),
        (2.2, 0),
        (2.5, 0),
    ]

    m = Map()
    m.create_map(32, 8, 8)
    startPoint = Point(-3, -4)
    endPoint = Point(1.0547,     -1.6986)

    for blockPoint in MAP_BLOCK_LIST:
        m.add_block(Point(blockPoint[0], blockPoint[1]))

    tr = m.get_trajectory(startPoint, endPoint)
    print(tr)

    write_on_plt(m, tr)
