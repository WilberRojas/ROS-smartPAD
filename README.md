# ROS smartPAD 2

![kcpv2](https://user-images.githubusercontent.com/74274632/197433000-a3bf6411-7843-4ee9-99c5-a092ee5f6e36.png)

### Updates:

* Button to clear Whiteboard from Ros Smartpad.
+ RVIZ show the points saved in a matrix of circles, witch blue means PTP point, green means LIN point, and red means no point saved.
+ now it is easier to save the points with:

  NEXT --> SAVE POINT

  Instead of:

  Select combo box --> SAVE POINT

+ The line of drawing now is continuous.
+ There is a menu that can show or hide different sections like: Movement, history, sequence, tools.
+ Save a sequence now is posible with NEW import, export buttons

### Instrucciones:

Compilar:
```
catkin_make
```
Actualizar la terminal:
```
source devel/setup.bash
```

correr en dos terminales diferentes:
1. Abrimos el escenario del robot:
```
roslaunch kr6_moveit kr6.launch
```
  En caso de no abrirse, en las opciones de rviz seleccione "open config" y abra el siguiente archivo:

![image](https://user-images.githubusercontent.com/74274632/197433633-47d77fb7-baa3-4319-baf4-03867c39230e.png)
![gdfgfd](https://user-images.githubusercontent.com/74274632/197435259-7168aafa-bb83-42ce-be2a-064415a1fd2d.png)

2. Ahora abrimos el control ROS SmartPAD
```
rosrun kcp interfaz.py 
```

![kcp](https://user-images.githubusercontent.com/74274632/197435573-6578618c-08a2-470f-bd70-2fe181e4d982.png)

podemos importar la secuencia demo llamada "sequence" para probar el robot.

### Ideas:
* Opcion para cambiar el color y grosor de la linea
* Distancia del extremo del kuka al marcador configurable en el control
* Realizar el spawn del marcador en RVIZ
* Exportar a KRL
* Cambiar color de objetos en RVIZ
* Añadir un ejercicio de "pick and place"
