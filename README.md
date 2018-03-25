# Interface matérielle avec les servomoteurs Dynamixel.

Interface entre les servomoteurs Dynamixel et `Ros`, et entre `Ros` et `Roscontrol`. Implémente le contrôle en vélocité des dynamixel. 

Dans un premier temps cette interface matérielle sera adaptée uniquement aux dynamixels du poignet de `Sara 2.0`.

## Dependencies

- Installer `ROS Control` :

```shell
sudo apt install ros-kinetic-ros-control -y
```

## Installation

- Authoriser votre utilisateur à utiliser la communication série :

```shell
sudo adduser your_user dialout
```

- Installer `git` :

```shell
sudo apt install git
```

- Cloner ce repo dans le workspace ros `ros_ws/src/`  :

```shell
cd ~/ros_ws/src
git clone https://github.com/WalkingMachine/wm_dynamixel.git
```
- Compiler le workspace :

```shell
cd ~/ros_ws
catkin_make
```

- Régler les paramètres de connexion avec le dynamixel dans `wm_dynamixel_node/launch/wm_dynamixel_node.launch`.

```shell
gedit ~/ros_ws/src/wm_dynamixel_hardware_interface/wm_dynamixel_node/launch/wm_dynamixel_node.launch
```
*Il est possible d'utiliser le [UDEV](https://github.com/WalkingMachine/sara_udev) de `Sara` pour garantir la connexion du contrôleur à la bonne place à chaque démarrage/connexion du connecteur USB.*

## Utilisation

**Voir section suivante pour plus de details sur les topics**

- Partir le node `wm_dynamixel_node`.

```shell
rosrun wm_dynamixel_node wm_dynamixel_node
```

ou, pour remplacer les valeurs d'initialisation par défaut:

```shell
roslaunch wm_dynamixel_node wm_dynamixel_node.launch
```

- Initialiser les dynamixels à utiliser en publiant sur le topic `dynamixel_init`.

- Envoyer des commandes de vélocité (ou position selon le mode de commande) sur le topic `dynamixel_cmd`.

- Lire la position des dynamixels sur le topic `dynamixel_pos`.


## Configuration YAML
```shell
HARDWARE_NAME:
  type: wm_dynamixel_hardware_interface/WMDynamixelHardwareInterface
  joints:
    - HARDWARE_NAME
  id: DYNAMIXEL_ID
  offset: OFFSET [rad]
  direction: DIRECTION [+1/-1]
  simulation: [true/false] #Fait abstraction du dynamixel (pour des tests)
  resolution: DYNAMIXEL_RESOLUTION #Voir Data sheet du dynamixel
  max_speed: VELOCITY_MAX_IN_POS_MODE #Environ vitesse moyenne en mode position puisqu'il atteint rapidement sa max_speed
  mode: COMMAND_MODE # velocitymode = 0 / position mode = 1
```
## Topics 

Tout les topics utilisent le message ros standard [`Float64MultiArray`](http://docs.ros.org/api/std_msgs/html/msg/Float64MultiArray.html).

|    Nom du topic    |         `dynamixel_init`         |                             `dynamixel_cmd`                            |                         `dynamixel_pos`                         |
|:------------------:|:--------------------------------:|:----------------------------------------------------------------------:|:---------------------------------------------------------------:|
| Index de la donnée | Sers à initialiser un dynamixel. | Sers à commander la vélocité ou la position d'un dynamixel initialisé. | Sers à lire le feedback en position des dynamixels initialisés. |
|          0         |          ID du dynamixel         |                             ID du dynamixel                            |                         ID du dynamixel                         |
|          1         |        Offset du dynamixel       |         Nouvelle vitesse *[rad/s]* OU Nouvelle position *[rad]*        |                       Angle actuel *[rad]*                      |
|          2         |      Resolution du dynamixel     |                                   */*                                  |                               */*                               |
|          3         |      Direction du dynamixel      |                                   */*                                  |                               */*                               |
|          4         |       Mode (0:vel ; 1:pos)       |                                   */*                                  |                               */*                               |
|          5         |               Ratio              |                                   */*                                  |                               */*                               |
|          6         |          Vitesse maximum         |                                   */*                                  |                               */*                               |

## Credits

- [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)

## License

MIT
