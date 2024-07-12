import dataclasses
import math


@dataclasses.dataclass
class Point:
    x: float
    y: float


class FlightPlanner:

    @classmethod
    def create_snake_traectory(cls, start_point: Point, fin_point: Point, num_x_point: int = 2, num_y_point: int = 2):
        """
        Функция для генерации списка точек в виде змейки
        :param start_point: Point(x, y)     - стартовая точка траектории
        :param fin_point: Point(x, y)       - конечная точка траектории
        :param num_x_point: int             - количество промежуточных точек вдоль оси Х
        :param num_y_point: int             - количество промежуточных точек вдоль оси У
        :return: list[Point]                - список точек для полета по траектории
        """

        traektory = []

        dx = (fin_point.x - start_point.x) / num_x_point
        dy = (fin_point.y - start_point.y) / num_y_point

        traektory.append(start_point)
        for x_point in range(num_x_point):
            for y_point in range(num_y_point):
                last_point = traektory[-1]
                if x_point % 2 == 0:
                    y = last_point.y + dy
                else:
                    y = last_point.y - dy

                traektory.append(Point(last_point.x, y))

            last_point = traektory[-1]
            x = last_point.x + dx
            traektory.append(Point(x, last_point.y))

        return traektory

    @classmethod
    def createCircleTr(cls, point: Point, radius: float, numPoints: int):
        """

        :param point:
        :param radius:
        :param numPoints:
        :return:
        """

    @staticmethod
    def checkDist(p1, p2, dist):
        d = math.dist(p1, p2)
        return True if d <= dist else False



