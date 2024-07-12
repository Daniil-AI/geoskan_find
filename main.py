import dataclasses
import time
import cv2
import threading

from robots.geobot_sdk import GeobotClient
from robots.pioneer_sdk import Pioneer
from pioneer_sdk import Camera

from tools.snake import *
from tools.NN import *
from tools.a_star import *


def drone_start() -> None:
    drone.arm()
    drone.takeoff()
    time.sleep(2)


def drone_end() -> None:
    drone.land()
    time.sleep(2)
    drone.disarm()


def drone_task(d_x, d_y, d_z) -> None:
    drone.go_to_local_point(d_x, d_y, d_z)
    while not drone.point_reached():
        time.sleep(0.1)
        pass


def get_position() -> float:
    while True:
        position = drone.get_local_position_lps()
        try:
            d_x, d_y, d_z = position[0], position[1], position[2]
            print(f"Полученная позиция x:{d_x}, z:{d_y}, z:{d_z}")
            break
        except TypeError:
            print("Получен None")

    return d_x, d_y, d_z


# Создание дронов и роботов
drone = Pioneer(ip="127.0.0.1", mavlink_port=8000)
bot = GeobotClient(ip="127.0.0.1", mavlink_port=8001)

# параметры карты
MIN_X, MAX_X = -4, 4
MIN_Y, MAX_Y = -4, 4
WORK_HEIGHT = 1.7

FIND_OBJ = False
# координаты обьекта (можно получить из дрона дроном можно указать руками)
object_pos = [0, 0]


def get_help():
    '''
    Cкрипт работы наземного бота в формате функции
    '''
    global MIN_X, MAX_X, MIN_Y, MAX_Y, WORK_HEIGHT
    global FIND_OBJ, object_pos

    GET_LAST_POINT = False

    # препятствия с ортофотоплана вносить ручками
    MAP_BLOCK_LIST = [
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
        (-0.64, -3.91),
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

    while True:

        # если нашли обьект
        if FIND_OBJ and not GET_LAST_POINT:
            # получим стартовые координаты
            #bot_start_x, bot_start_y, bot_start_z = bot.get_local_position_lps()
            _, _, bot_start_z =  bot.get_local_position_lps()
            bot_start_x, bot_start_y = -3, -4

            # создание карты пространства c обьектом интереса
            m = Map()
            m.create_map((MAX_X+MAX_Y)*2, MAX_X*2, MAX_Y*2)
            startPoint = Point(bot_start_x, bot_start_y)
            endPoint = Point(object_pos[0], object_pos[1])

            for blockPoint in MAP_BLOCK_LIST:
                m.add_block(Point(blockPoint[0], blockPoint[1]))

            # после нахождения отпимального пути начнем движение
            tr = m.get_trajectory(startPoint, endPoint)
            for point in tr:
                bot.go_to_local_point(point.x, point.y, bot_start_z)
                while not bot.point_reached():
                    time.sleep(0.1)

            GET_LAST_POINT = True

        # достигли точки идем спать
        if GET_LAST_POINT:
            bot.emercy_detection()

            # зарисуем траекторию движения бота
            write_on_plt(m, tr)
            time.sleep(5)
            break

        time.sleep(0.02)


def search():
    '''
    Cкрипт работы дрона
    '''
    global MIN_X, MAX_X, MIN_Y, MAX_Y, WORK_HEIGHT
    global FIND_OBJ, object_pos

    # запустим дрон и подождем
    drone_start()

    # получим позицию дрона
    start_x, start_y, start_z = get_position()

    # поднимемся на высоту на той же позиции
    drone_task(start_x, start_y, WORK_HEIGHT)

    # генерация курса
    tr = FlightPlanner.create_snake_traectory(Point(MAX_X, MIN_Y), Point(MIN_X, MAX_Y), 10, 10)
    camera = Camera()

    while True:
        if FIND_OBJ:
            # горим
            drone.led_control(r=200, g=30, b=30)
            time.sleep(5)

            # построим обратную траекторию
            x, y, z = get_position()
            tr = FlightPlanner.create_snake_traectory(Point(x, y), Point(start_x, start_y), 3, 3)

            for point in tr:
                drone_task(point.x, point.y, WORK_HEIGHT)

            # cнижение
            drone_task(start_x, start_y, WORK_HEIGHT/2)
            drone_task(start_x, start_y, WORK_HEIGHT/4)
            drone_end()
            break

        else:
            # поиск обьекта
            for point in tr:
                drone_task(point.x, point.y, WORK_HEIGHT)
                frame = camera.get_cv_frame()
                cv2.imshow("STEP_IMAGE", frame)
                find, pos_x, pos_y = calculate_coordinates(frame)

                if find:
                    FIND_OBJ = True
                    object_pos = [pos_x, pos_y]
                    break


if __name__ == "__main__":
    # Запуск функций в отдельных потоках
    th1 = threading.Thread(target=search)
    th2 = threading.Thread(target=get_help)

    th1.start()
    th2.start()

    cv2.destroyAllWindows()
