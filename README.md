# ROS smartPAD Simulation

Despues de clonar el repositorio, compilelo con:

```
catkin_make
```
Luego actualice la terminal para que pueda ver los packages compilados:
```
source devel/setup.bash
```
1. Ahora correremos la simulacion de RVIZ:
```
roslaunch kr6_moveit kr6.launch
```
![image](https://user-images.githubusercontent.com/74274632/195968323-06e21508-8c71-4fd5-a5f3-be2323bc8766.png)

2. En otra terminal correremos el ROS smartpad:
```
rosrun kcp kcp.py
```
![image](https://user-images.githubusercontent.com/74274632/195968235-7fdbfff6-000a-4247-b761-246dc4b4b5ca.png)


3. Adicionalmente, para usar la pizarra virtual deberemos correr:

```
rosrun kcp whiteboard.py
```
No desplegara ninguna ventana, se puede visualizar directamente en RVIZ:

![image](https://user-images.githubusercontent.com/74274632/195968453-0f0f381b-517f-446b-b68a-e93ed5c56e22.png)
