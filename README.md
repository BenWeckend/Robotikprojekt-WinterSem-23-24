# Ros2 projekte:
Befor man die Projekte ausführt musst man jeweils dies in das Terminal eingeben:
```console
colcon build
```
### Pub and Sub:
Klone "my_package" in den 'src'-Ordner deines Arbeitsverzeichnisses. Navigiere anschließend in dein Arbeitsverzeichnisses und öffne ein Terminal. Gib folgende Befehle ein:

```console
source install/setup.bash 
ros2 run pub_sub pub
```
in einem anderem Terminal:
```console
source install/setup.bash 
ros2 run pub_sub sub
```
Dann müsste eine Übertragung stattfinden.


Bevor man die jetzt folgenden packages mit ```console ros2 run ``` ausführt muss man sich mit beispielsweise ```console ros2 topic list ``` sicher stellen, dass man auch mit dem Turtlebot verbunden ist.
### Lidar:
Füge "lidar_python" zu dein Arbeitsverzeichniss hinzu. Öffne ein Terminal und navigiere anschließend in dein Arbeitsverzeichnisses.
Gib folgenden Befehl ein:
```console
ros2 run lidar_python my_node
```
Nun müsste der Turtlebot dem Objekt sich stetig nähern/folgen, welches am nähsten ist.
### Camera
Füge "line_python" zu dein Arbeitsverzeichniss hinzu. Öffne ein Terminal und navigiere anschließend in dein Arbeitsverzeichnisses.
Stelle den Turtlebot auf die weiße Linien welche es zu folgen gilt und gib folgenden Befehl ein:
```console
ros2 run line_python my_node
```
