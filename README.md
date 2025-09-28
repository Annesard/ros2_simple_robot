# Домашняя работа 2: ROS2 Пакет с URDF Моделью Робота
Автор: Анастасия Рябкова
ROS Дистрибутив: Rolling
OS: Ubuntu 24

##  Структура робота
### Звенья (Links):
* base_link - базовое звено (синий цилиндр)
* middle_link - среднее звено (красный параллелепипед)
* upper_link - верхнее звено (зеленый параллелепипед)
* effector_link - конечный эффектор (желтая сфера)

## Шарниры (Joints):
* base_to_middle - вращение вокруг оси Z (±3.14 rad)
* middle_to_upper - вращение вокруг оси Y (±2.0 rad)
* upper_to_effector - вращение вокруг оси Y (±2.0 rad)

## Архитектура системы
### Компоненты:
* URDF модель (my_robot.urdf) - описание геометрии и кинематики робота
* Robot Controller (robot_controller.py) - управление движением шарниров
* Launch файл (my_robot.launch.py) - запуск всей системы
* RViz конфигурация (view_robot.rviz) - настройки визуализации

### Узлы ROS2:
* robot_state_publisher - публикация состояния робота
* robot_controller - управление движением суставов
* rviz2 - визуализация модели

## Установка и запуск
### Сборка пакета
```
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
```

## Запуск
```
ros2 launch my_robot_description my_robot.launch.py
```

