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
