ROS2 INSTALADO EN THINKPAD
UBUNTU 22.04
ROS2 Humble; instalación: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
Ver versión de ROS instalada: printenv ROS_DISTRO
Checar variables de entorno: env
IMPORTANTE DE DESACTIVAR EL ENTORNO DE ANACONDA: conda deactivate
VERSION DE UBUNTU 22.04
ROS2
printenv ROS_DISTRO

En este tutorial de ROS, se explica como modelar y simular un robot móvil de 4 ruedas en ROS2 y Gazebo desde scratch. Explicamos como escribir código xacro, gazebo y códigos de lanzamiento en Python para el modelo y simulado de modelo. Tambien, se explica como controlar el robor usando las teclas del teclado.
Empezar iniciando el ambiente: source /opt/ros/humble/setup.bash
Despues necesitamos instalar los paquetes de ROS2 y Gazebo. Algunos de esos paquetes puede que ya los tengas, como sea, es buena idea el tratar de instalarlos. Si ya los tienes, el intento de instalación no cambiara nada en el sistema:

sudo apt-get update
sudo apt-get install gedit
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro

sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros-core
sudo apt install ros-humble-geometry2
sudo apt-get install ros-humble-gazebo-msgs
sudo apt-get install ros-humble-gazebo-plugins

sudo apt-get install ros-humble-ros-ign-bridge
sudo apt-get install ros-humble-teleop-twist-keyboard

Despues, en orden de checar la instalación de gazebo. Necesitamos escribir
gazebo

Tambien se puede checar la versión de Gazebo con:
gazebo --version

CREAR UN WORKSPACE Y EL PAQUETE
Cerrar las terminales que estaban abiertas, abrir una nueva terminal y source al entorno
source /opt/ros/humble/setup.bash

Después, crear las carpetas de trabajo
mkdir -p ~/ws_mobile/src

Aqui, ws es abreviacion de “workspace”, y mobile es el nombre. Aquí, estaremos creando 2 carpetas. La primera carpeta es “ws_mobile” y dentro de esta carpeta, nosotros hemos creado otra carpeta llamada “src”. Luego, creamos y construiremos el workspace
cd ~/ws_mobile/
colcon build

Despues, necesitaríamos crear el paquete:
cd ~/ws_mobile/src
ros2 pkg créate –build-type ament_cmake mobile_robot

El nombre del paquete es “mobile_robot”. El paquete fuente de archivos y carpetas estará en la carpeta ~/ws_mobile/src/mobile_robot/ . Consecuentemente, hay que dirigirnos a ese folder que nos dejara crear varias subcarpetas:
cd ~/ws_mobile/src/mobile_robot/
mkdir launch model

Aqui, hemos creado 2 carpetas: launch y model. La carpeta launch contiene el archivo Python launch necesario para ejecutar el modelo en gazebo. La carpeta modelo va a contener el archivo fuente Xacro y URDF del modelo, asi como un modelo de un mundo en Gazebo vacio que va a ser usado en el entorno. Nota que las siglas URDF es por “United Robotics Description Format”. Este es el formato del archivo que es usado para especificar la geometría de los robots y de algunas otras propiedades. Xacro es un XML macro lenguaje usado para parametizar y simplificar el desarrollo de modelos URDF. Ahora vamos a construir el workspace una vez mas. Para construir el workspace y los paquetes, tienes que navegar a la carpeta fuente del workspace/espacio de trabajo
cd ~/ws_mobile/
colcon build

CREAR EL ARCHIVO URDF(XACRO) y GAZEBO
Primero, vamos a crear el modelo del robot. Creamos el modelo del robot creando 2 archivos: Archivo Xacro y Gazebo. El primer archivo Xacro contiene el modelo 3D del robot, y el segundo. El archivo Gazebo contendrá los parámetros necesarios para representar satisfactoriamente y controlar el modelo en Gazebo. Primero, habrá que dirigirse a la carpeta modelo.
cd ~/ws_mobile/src/mobile_robot/model

Crear el archive modelo Xacro definiendo la geometría:
gedit robot.xacro

y pon el contenido del archivo:
<?xml version="1.0"?>

<robot name="differential_drive_robot" xmlns:xacro="https://www.ros.org/wiki/xacro">

<!-- Body dimensions -->
<xacro:property name="body_link_x_dim" value="1"/>
<xacro:property name="body_link_y_dim" value="0.6"/>
<xacro:property name="body_link_z_dim" value="0.3"/>

<!-- Wheel dimensions -->
<xacro:property name="wheel_link_radius" value="0.15"/>
<xacro:property name="wheel_link_length" value="0.1"/>
<xacro:property name="wheel_link_z_location" value="-0.1"/>

<!-- Material density -->
<xacro:property name="body_density" value="2710.0"/>
<xacro:property name="wheel_density" value="2710.0"/>

<!-- Pi constant -->
<xacro:property name="pi_const" value="3.14159265"/>

<!-- Robot body and wheel mass -->
<xacro:property name="body_mass" value="${body_density*body_link_x_dim*body_link_y_dim*body_link_z_dim}"/>
<xacro:property name="wheel_mass" value="${wheel_density*pi_const*wheel_link_radius*wheel_link_radius*wheel_link_length}"/>

<!-- Moments of inertia of the wheel -->
<xacro:property name="Iz_wheel" value="${0.5*wheel_mass*wheel_link_radius*wheel_link_radius}" />
<xacro:property name="I_wheel" value="${(1.0/12.0)wheel_mass(3.0*wheel_link_radius*wheel_link_radius+wheel_link_length*wheel_link_length)}" />

<!-- This macro defines the complete inertial section of the wheel -->
<!-- It is used later in the code -->

<xacro:macro name ="inertia_wheel">
	<inertial>
	<origin rpy="0 0 0" xyz="0 0 0"/>
	<mass value="${wheel_mass}"/>
	<inertia ixx="${I_wheel}" ixy="0.0" ixz="0.0" iyy="${I_wheel}" iyz="0" izz="${Iz_wheel}" />
	</inertial>
</xacro:macro>	

<!-- Se incluye el archivo que define opciones extra de Gazebo y driver de control de movimientos -->
<xacro:include filename="$(find mobile_robot)/model/robot.gazebo" />

<!-- SE DEFINEN LOS LINKS/JOINTS -->
<!-- ___________________ -->

<link name="dummy">
</link>
<joint name="dummy_joint" type="fixed">
	<parent link="dummy"/>
	<child link="body_link"/>
</joint>

<!-- START : BODY LINK OF THE ROBOT -->
<!-- __________________ -->

<link name="body_link">
	<visual>
		<geometry>
			<box size="${body_link_x_dim} ${body_link_y_dim} ${body_link_z_dim}" />
		</geometry>
	<origin rpy="0 0 0" xyz="0 0 0"/>
	</visual>	

	<collision>
		<geometry>
			<box size="${body_link_x_dim} ${body_link_y_dim} ${body_link_z_dim}" />
		</geometry>
		<origin rpy="0 0 0" xyz="0 0 0"/>
	</collision>
	
	<inertial>
		<origin rpy="0 0 0" xyz="0 0 0"/>
		<mass value="${body_mass}"/>
	<inertia
	ixx="${(1/12)body_mass(body_link_y_dim*body_link_y_dim+body_link_z_dim*body_link_z_dim)}"
	ixy="0" ixz="0" iyy="${(1/12)body_mass(body_link_x_dim+body_link_x_dim*body_link_z_dim*body_link_z_dim)}"	
	iyz="0"
	izz="${(1/12)body_mass(body_link_y_dim*body_link_y_dim+body_link_x_dim*body_link_x_dim)}"/>
	</inertial>
</link>	

<!-- _________________ -->
<!-- END: BODY LINK OF THE ROBOT -->
<!-- _________________ -->

<!-- _________________ -->
<!-- START: BACK RIGHT WHEEL OF THE ROBOT AND THE JOINT -->
<!-- _________________ -->

<joint name="wheel1_joint" type="continuous" >
	<parent link="body_link"/>
	<child link="wheel1_link"/>
	<origin xyz="${-body_link_x_dim/2+1.2*wheel_link_radius} ${-body_link_y_dim/2-wheel_link_length/2} ${wheel_link_z_location}" rpy="0 0 0" />
	<axis xyz="0 1 0"/>
	<limit effort="1000" velocity="1000"/>
	<dynamics damping="1.0" friction="1.0"/>
</joint>

<link name="wheel1_link">
	<visual>
		<origin rpy="1.570795 0 0" xyz="0 0 0"/>
		<geometry>
			<cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
		</geometry>
	</visual>				
	
	<collision>
		<origin rpy="1.570795 0 0" xyz="0 0 0"/>
		<geometry>
			<cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
		</geometry>
	</collision>
	
	<xacro:inertia_wheel/>	
</link>

<!-- _________________ -->
<!-- END: BACK RIGHT WHEEL OF THE ROBOT AND THE JOINT -->
<!-- _________________ -->		
	
	
<!-- _________________ -->
<!-- START: BACK LEFT WHEEL OF THE ROBOT AND THE JOINT -->
<!-- _________________ -->	
	
<joint name="wheel2_joint" type="continuous" >
	<parent link="body_link"/>
	<child link="wheel2_link"/>
	<origin xyz="${-body_link_x_dim/2+1.2*wheel_link_radius} ${body_link_y_dim/2+wheel_link_length/2} ${wheel_link_z_location}" rpy="0 0 0"/>
	<axis xyz="0 1 0"/>
	<limit effort="1000" velocity="1000"/>
	<dynamics damping="1.0" friction="1.0"/>
</joint>

<link name="wheel2_link">
	<visual>
		<origin rpy="1.570795 0 0" xyz="0 0 0"/>
		<geometry>
			<cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
		</geometry>
	</visual>
	
	<collision>
		<origin rpy="1.570795 0 0" xyz="0 0 0"/>
		<geometry>
			<cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
		</geometry>
	</collision>
	<xacro:inertia_wheel/>
</link>						
	
<!-- _________________ -->
<!-- END: BACK LEFT WHEEL OF THE ROBOT AND THE JOINT -->
<!-- _________________ -->


<!-- _________________ -->
<!-- START: FRONT RIGHT WHEEL OF THE ROBOT AND THE JOINT -->
<!-- _________________ -->

<joint name="wheel3_joint" type="continuous" >
	<parent link="body_link"/>
	<child link="wheel3_link"/>
	<origin xyz="${body_link_x_dim/2-1.2*wheel_link_radius} ${-body_link_y_dim/2-wheel_link_length/2} ${wheel_link_z_location}" rpy="0 0 0" />
	<axis xyz="0 1 0" />
	<limit effort="1000" velocity="1000"/>
	<dynamics damping="1.0" friction="1.0"/>
</joint>

<link name="wheel3_link">
	<visual>
		<origin rpy="1.570795 0 0" xyz="0 0 0"/>
		<geometry>
			<cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
		</geometry>	
	</visual>
	<collision>
		<origin rpy="1.570795 0 0" xyz="0 0 0"/>
			<geometry>
				<cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
			</geometry>
		</collision>
	<xacro:inertia_wheel/>
</link>				

<!-- _________________ -->
<!-- END: FRONT RIGHT WHEEL OF THE ROBOT AND THE JOINT -->
<!-- _________________ -->

<!-- _________________ -->
<!-- START: FRONT LEFT WHEEL OF THE ROBOT AND THE JOINT -->
<!-- _________________ -->

<joint name="wheel4_joint" type="continuous">
	<parent link="body_link"/>
	<child link="wheel4_link"/>
	<origin xyz="${body_link_x_dim/2-1.2*wheel_link_radius} ${body_link_y_dim/2+wheel_link_length/2} ${wheel_link_z_location}" rpy="0 0 0"/>
	<axis xyz="0 1 0"/>
	<limit effort="1000" velocity="1000"/>
	<dynamics damping="1.0" friction="1.0"/>
</joint>

<link name="wheel4_link">
	<visual>
		<origin rpy="1.570795 0 0" xyz="0 0 0"/>
		<geometry>
			<cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
		</geometry>
	</visual>
	
	<collision>
		<origin rpy="1.570795 0 0" xyz="0 0 0"/>
		<geometry>
			<cylinder length="${wheel_link_length}" radius="${wheel_link_radius}"/>
		</geometry>
	</collision>
	<xacro:inertia_wheel/>
</link>
<!-- _________________ -->
<!-- END: FRONT LEFT WHEEL OF THE ROBOT AND THE JOINT -->
<!-- _________________ -->			
</robot>

Después, crear un archivo Gazebo adicional que esta incluido en el archivo Xacro:
gedit robot.gazebo

<?xml version="1.0"?>
<robot>

<!-- Everything is described here -->
<!-- http://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros -->
<!-- mu1 and mu2 are friction coefficients -->

<gazebo reference="body_link">
<mu1>0.2</mu1>
<mu2>0.2</mu2>
<material>Gazebo/Red</material>
</gazebo>

<gazebo reference="wheel1_link">
<mu1>0.2</mu1>
<mu2>0.2</mu2>
<material>Gazebo/Yellow</material>
</gazebo>

<gazebo reference="wheel2_link">
<mu1>0.2</mu1>
<mu2>0.2</mu2>
<material>Gazebo/Yellow</material>
</gazebo>

<gazebo reference="wheel3_link">
<mu1>0.2</mu1>
<mu2>0.2</mu2>
<material>Gazebo/Yellow</material>
</gazebo>

<gazebo reference="wheel4_link">
<mu1>0.2</mu1>
<mu2>0.2</mu2>
<material>Gazebo/Yellow</material>
</gazebo>

<!-- Controller for the 4-wheeled robot -->
<gazebo>
<plugin name='skid_steer_drive' filename='libgazebo_ros_diff_drive.so'>

	<ros>
		<namespace></namespace>
	</ros>
	
	<!-- Number of wheel pairs -->
	<num_wheel_pairs>2</num_wheel_pairs>
	
	<!-- wheels0 -->
	<left_joint>wheel4_joint</left_joint>
	<right_joint>wheel3_joint</right_joint>
	
	<!-- wheels1 -->
	<left_joint>wheel2_joint</left_joint>
	<right_joint>wheel1_joint</right_joint>
	
	<!-- kinematics -->
	<wheel_separation>${body_link_y_dim+wheel_link_length}</wheel_separation>
	<wheel_separation>${body_link_y_dim+wheel_link_length}</wheel_separation>
	
	<wheel_diameter>${wheel_link_radius}</wheel_diameter>
	<wheel_diameter>${wheel_link_radius}</wheel_diameter>
	
	<!-- limits -->
	<max_wheel_torque>1000</max_wheel_torque>
	<max_wheel_acceleration>5.0</max_wheel_acceleration>
	
	<!-- output -->
	<publish_odom>true</publish_odom>
	<publish_odom_tf>true</publish_odom_tf>
	<publish_wheel_tf>true</publish_wheel_tf>
	
	<odometry_frame>odom</odometry_frame>
	<robot_base_frame>dummy</robot_base_frame>
	
	</plugin>
</gazebo>

</robot>

-	Guardar y cerrar




CREAR EL ARCHIVO DE PYTHON PARA LANZAR
Creamos el archivo de lanzamiento en la carpeta “launch”. Vamos a dirigirnos a la ruta:

cd ~/ws_mobile/src/mobile_robot/launch
Crear y editar el archivo
gedit gazebo_model.launch.py
Contenido de archivo de Python:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():

	#Este nombre tiene que coincidir con el nombe del robot en el archivo Xacro
	robotXacroName='differential_drive_robot'
	
	#Este es el nombre de nuestro paquete, al mismo tiempo este es el nombre de la carpeta que será usada para definir el path
	namePackage='mobile_robot'
	
	#Este es la ruta relativa a el archivo xacro definiendo el modelo
	modelFileRelativePath = 'model/robot.xacro'
	
	#Este es la ruta relativa a el archivo de Gazebo World
	worldFileRelativePath = 'model/empty_world.world'
	
	#Esta es la ruta absoluta al modelo
	pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
	
	#Esta es la ruta absoluta al modelo del mundo
	pathWorldFile = os.path.join(get_package_share_directory(namePackage),worldFileRelativePath)
	
	#Obtener la descricion del robot desde el archivo del modelo xacro
	robotDescription = xacro.process_file(pathModelFile).toxml()
	
	
	
	#Este es el archivo para despliegue desde el paquete de gazebo_ros
	gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),
								'launch','gazebo.launch.py'))
	
	#Esta es la descripcion del despliegue
	gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch,launch_arguments={'world': pathWorldFile}.items())
	
	#Aqui, creamos un Nodo gazebo_ros
	spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py',
				arguments=['-topic','robot_description','-entity', robotXacroName],output='screen')
	
	#Nodo Robot State Publisher
	
	nodeRobotStatePublisher = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': robotDescription,
		'use_sim_time': True}]
	)
	
	#Aqui creamos una descripcion vacia del lanzamiento del objeto
	launchDescriptionObject = LaunchDescription()
	
	#Se añade gazeboLaunch
	launchDescriptionObject.add_action(gazeboLaunch)
	
	#Añadimos los 2 nodos
	launchDescriptionObject.add_action(spawnModelNode)
	launchDescriptionObject.add_action(nodeRobotStatePublisher)
	
	return launchDescriptionObject

Aqui, hay que asegurarnos de alinear bien los tabs con las funciones de Python y por los ciclos. Si no se alinean apropiadamente los tabs entonces es probable que obtengas algún error al construir el paquete. Consecuentemente, es mejor idea usar otro editor de 

AJUSTES FINALES Y CORRER EL MODELO
Debemos de añadir las dependencias
cd ~/ws_mobile/src/mobile_robot
gedit package.xml

Añade estas lineas a “package.xml”

<exec_depend>joint_state_publisher</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>gazebo_ros</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>



PACKAGE.xml --- COMPLETO

<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mobile_robot</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="dcd@todo.todo">dcd</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

Después, necesitamos ajustar el archivo CMakeLists.txt, necesitamos decirle a ROS2 que los archivos importantes están en las carpetas “launch” y “model”
cd ~/ws_mobile/src/mobile_robot
gedit CMakeLists.txt

Despues, arriba de “if(BUILD_TESTING)”, escribe lo siguiente
install(
	DIRECTORY launch model
	DESTINATION share/${PROJECT_NAME}
)

Despues, tenemos que hacer un último build al espacio de trabajo y el paquete
cd ~/ws_mobile
colcon build

Despues tenemos que hacer un source al paquete, Eso es que tenemos que crear una capa superpuesta
source ~/ws_mobile/install/setup.bash

Finalmente, corremos gazebo
ros2 launch mobile_robot gazebo_model.launch.py

CONTROLAR EL ROBOT USANDO EL TECLADO
No cerrar la terminal que tiene el gazebo abierto. Abrir una terminal, y en la nueva terminal escribir:
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Esto hará que se “emparejen” los entornos y que empiece el nodo de teleop_twist_keyboard que es usado para controlar el robot.
