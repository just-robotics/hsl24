### Хакатон СтарЛайн 2024

#### Сборка решения

1. Установите ПО, указанное в [исходном задании](https://github.com/I1sh/hackathon_sl24/blob/master/docs/README.md).

2. Склонируйте репозиторий и перейдите в корневую директорию репозитория.

        git clone https://github.com/just-robotics/hsl24.git
        cd hsl24

3. Соберите и запустите докер контейнер с дополнительными зависимостями решения:

         cd simulation/docker; bash build.bash && bash run.bash

4. Запустите bash-сессию в конейнере:

        bash into.bash
   
5. Соберите проект:

		source /opt/ros/humble/setup.bash
	    colcon build 
		source ./install/setup.bash

#### Запуск решения
Запустите симуляцию:

        ros2 launch survey ozyland.launch.py
		
Запустите ноду qr детектора:

        ros2 run solution qr_detector
		
Запустите следующие launch-файлы:

        ros2 launch solution bringup.launch.py
        ros2 launch  solution navigation.launch.py
        

		
Дождитесь вывода Creating bond timer...  в navigation.launch.py, после запустите пакет исследователя:
        
        ros2 launch solution explore.launch.py

Для мониторинга кодов прослушайте топик:
		
        ros2 topic echo /result
