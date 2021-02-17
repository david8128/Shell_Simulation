# airsim_car_ros_pkgs
Un pequeño tutorial de cómo usar la herramienta airsim en Windows con el wrapper de ROS desarrollado por SWRi para la Shell Eco-Marathon 2021, corriendo ROS en WSL2. El tutorial está en desarrollo.
A ROS wrapper over the AirSim C++ client library. 

##  Setup 
### NOTA: Este repositorio debe ser clonado en la Carpeta AirSim/ros/src

### Descargar e instalar Unreal Engine desde Epic Games Launcher (para windows)
### Descargar y montar [AirSim para windows](https://microsoft.github.io/AirSim/build_windows/)
### Windows Subsytem for Linux on Windows 10
[Instrucciones para instalar WSL2 y ROS Melodic](https://jack-kawell.com/2020/06/12/ros-wsl2/)

Una vez configurado ejecutar los siguientes comandos en el terminal de WSL:
```bat
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0 >> ~/.bashrc
export WSL_HOST_IP=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}') >> ~/.bashrc
```

##  Build
1. Instalar algunos paquetes preliminares:
```bash
sudo apt update
sudo apt install build-essential git git-lfs gcc-8 g++-8 python-catkin-tools
```
2. Download and build Unreal Engine for Linux (on WSL): 
```bat
 cd $HOME/
 git clone -b 4.24 https://github.com/EpicGames/UnrealEngine.git
```
 - Pasar nombre de usuario y contraseña cuando lo pida. Una vez descargado todo navegamos hacia la carpeta y ejecutamos el script de setup. IMPORTANTE: NO USAR SUDO
```bat
 cd $HOME/UnrealEngine/
 ./Setup.sh
```
 - Una vez terminado se ejecuta Generate files:
```bat
 ./GenerateProjectFiles.sh
```
 - Y ahora compilar:
```bat
 make
```
Cualquier duda revisar el [instructivo oficial de Unreal](https://docs.unrealengine.com/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html)

2.1. (Opcional pero facilita la vida) Instalar ue4cli: 
 - en un terminal de WSL correr:
 ```bat
  sudo pip3 install ue4cli
 ```
 (Instalar pip3 si hace falta)
 - Añadir al PATH la ubicación de Unreal:
 ```bat
  export PATH=$PATH:$HOME/UnealEngine/Engine/Binaries/Linux/ >> ~/.bashrc
 ```
 2. Clone SwRI's AirSim fork:
   ```bash
   mkdir -p $HOME/src/
   cd $HOME/src
   git clone https://github.com/swri-robotics/AirSim.git
   ```

3. Clone this repository inside AirSim's Unreal/Environments directory:
   ```bash
   mkdir -p $HOME/src/AirSim/Unreal/Environments/
   cd $HOME/src/AirSim/Unreal/Environments/
   git clone https://github.com/swri-robotics/Neighborhood.git
   ```

4. Use git-lfs to grab the full contents of the directory:
```bash
cd $HOME/src/AirSim/Unreal/Environments/Neighborhood
git lfs pull
```

5. Build AirSim:
   ```bash
   cd $HOME/src/AirSim
   ./setup.sh
   ./build-neighborhood.sh
   ```

6. Build the Neighborhood environment:
   ```bash
   cd $HOME/src/AirSim/Unreal/Environments/Neighborhood
   ue4 build
   ```
## Building the ROS Node
# Preliminares: conectando el módulo AirSim de Windows con WSL y ROS
# Using AirSim ROS wrapper 
Ir al archivo (En WSL) $HOME\src\AirSim\ros\src\airsim_car_ros_pkgs\src\airsim_node.cpp y cambiar la línea 14 (creo) por su propia dirección IP
 ```cpp
   //std::string host_ip = "localhost";
    //Código para forzar que el wrapper de ROS mapee a windows Ejemplo esta es mi IP: 172.18.208.1 
    std::string host_ip = "172.18.208.1";
 ```
# Crear el catkin workspace (en WSL): 

1. Navegar hasta la carpeta donde está ros, ejecutar los comandos preliminares (cuando diga si quiere borrar las carpetas después de catkin clean diga yes)
   ```bash
   cd $HOME/src/AirSim/ros
   catkin clean
   . /opt/ros/melodic/setup.bash;   rosdep install src -y --from-paths -i;
   catkin init;   catkin config -e /opt/ros/melodic;   catkin config --install;   catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo;
   ```

2. Build it; note that it requires GCC 8:
   ```bash
   CC=gcc-8 CXX=g++-8 catkin build
   ```
## Correr
1. Copiar el archivo settings.json de este repositorio a la carpeta en donde está el binario (En este caso: D:\AirSim\Unreal\Environments\Neighborhood\settings.json)
2. Editar el archivo run.bat para que busque los settings en la carpeta donde está el binario:
```bat
start AirSimNH -windowed --settings 'D:\AirSim\Unreal\Environments\Neighborhood\settings.json'
```
3. Correr el entorno en AirSim para Windows.
4. Correr las siguientes líneas de código en un nuevo terminal de WSL
```bash
source devel/setup.bash;
roslaunch airsim_car_ros_pkgs airsim_node.launch output:=screen host:=$WSL_HOST_IP

```
Y en un nuevo terminal:
```bash
roslaunch airsim_car_ros_pkgs rviz.launch;
```

Rezar para que funcione. Contacto en juctorresme@unal.edu.co
