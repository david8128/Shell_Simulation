# airsim_car_ros_pkgs
Un pequeño tutorial de cómo usar la herramienta airsim en Windows con el wrapper de ROS desarrollado por SWRi para la Shell Eco-Marathon 2021, corriendo ROS en WSL2. El tutorial está en desarrollo.
A ROS wrapper over the AirSim C++ client library. 

##  Setup 
### Descargar e instalar Unreal Engine desde Epic Games Launcher
### Windows Subsytem for Linux on Windows 10
- WSL setup:
  * Get [Windows Subsystem for Linux](https://docs.microsoft.com/en-us/windows/wsl/install-win10)
  * Get [Ubuntu 16.04](https://www.microsoft.com/en-us/p/ubuntu-1604-lts/9pjn388hp8c9?activetab=pivot:overviewtab) or [Ubuntu 18.04](https://www.microsoft.com/en-us/p/ubuntu-1804-lts/9n9tngvndl3q?activetab=pivot%3Aoverviewtab)  
  * Go to Ubuntu 16 / 18 instructions!


- Setup for X apps (like RViz, rqt_image_view, terminator) in Windows + WSL
  * Install [Xming X Server](https://sourceforge.net/projects/xming/). 
  * Find and run `XLaunch` from the Windows start menu.   
  Select `Multiple Windows` in first popup, `Start no client` in second popup, **only** `Clipboard` in third popup. Do **not** select `Native Opengl`.  
  * Open Ubuntu 16.04 / 18.04 session by typing `Ubuntu 16.04`  / `Ubuntu 18.04` in Windows start menu.  
  * Recommended: Install [terminator](http://www.ubuntugeek.com/terminator-multiple-gnome-terminals-in-one-window.html) : `$ sudo apt-get install terminator.` 
    - You can open terminator in a new window by entering `$ DISPLAY=:0 terminator -u`. 

##  Build
- Build AirSim (on WSL)
```
git clone https://github.com/Microsoft/AirSim.git;
cd AirSim;
./setup.sh;
./build.sh;
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
1. Navegar hasta la carpeta donde está ros, ejecutar los comandos preliminares
   ```bash
   cd $HOME/src/AirSim/ros;   . /opt/ros/melodic/setup.bash;   rosdep install src -y --from-paths -i;
   catkin init;   catkin config -e /opt/ros/melodic;   catkin config --install;   catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo;
   ```

2. Build it; note that it requires GCC 8:
   ```bash
   CC=gcc-8 CXX=g++-8 catkin build
   ```
## Correr
1. Copiar el archivo settings.json de este repositorio a la carpeta Documentos/Airsim (Falta corregir por qué no detecta el archivo)
2. Correr el entorno en AirSim para Windows (Hasta ahora no se ha probado si funciona con los binarios precompilados).
3. Correr las siguientes líneas de código en un nuevo terminal de WSL
```bash
source devel/setup.bash;
roslaunch airsim_car_ros_pkgs airsim_node.launch output:=screen host:=$WSL_HOST_IP

```
Y en un nuevo terminal:
```bash
roslaunch airsim_car_ros_pkgs rviz.launch;
```

Rezar para que funcione. Contacto en juctorresme@unal.edu.co
