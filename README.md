# dashgo_car

首先新建个工作空间，将ros的navigation包编译，过程中可能会出现多个依赖项缺失问题，参考https://blog.csdn.net/Groot_Lee/article/details/79202507

然后是dashgo包，执行dashgo_driver文件夹下的create_dashgo_udev.sh，为小车串口添加别名。如果包是拷贝的，有些py文件可能需要chmod

使用hokuyo激光时，需要执行包里create_hokuyo_udev.sh文件，为激光串口附别名

使用rosserial打包lib时，apt安装的可能提示缺少make_library.py文件，可能自己下个rosserial包编译

simple_navigation_goals包是机器人多点导航的python版本，c++版本参考patrol_robot包
