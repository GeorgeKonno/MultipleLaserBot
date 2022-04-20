####### add protobuf lib path ########/home/qjny/local_install/lib
export PATH=/home/gliu/Desktop/workspace_GEORGEBOT/src/MultipleLaserBot/compile_dependency/usr/local/bin/:$PATH
#(动态库搜索路径) 程序加载运行期间查找动态链接库时指定除了系统默认路径之外的其他路径
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/gliu/Desktop/workspace_GEORGEBOT/src/MultipleLaserBot/compile_dependency/usr/local/lib/
#(静态库搜索路径) 程序编译期间查找动态链接库时指定查找共享库的路径
export LIBRARY_PATH=$LIBRARY_PATH:/home/gliu/Desktop/workspace_GEORGEBOT/src/MultipleLaserBot/compile_dependency/usr/local/lib/
#执行程序搜索路径
export PATH=$PATH:/home/gliu/Desktop/workspace_GEORGEBOT/src/MultipleLaserBot/compile_dependency/usr/local/bin/
#c程序头文件搜索路径
export C_INCLUDE_PATH=$C_INCLUDE_PATH:/home/gliu/Desktop/workspace_GEORGEBOT/src/MultipleLaserBot/compile_dependency/usr/local/include/
#c++程序头文件搜索路径
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/home/gliu/Desktop/workspace_GEORGEBOT/src/MultipleLaserBot/compile_dependency/usr/local/include/
#pkg-config 路径
export PKG_CONFIG_PATH=/home/gliu/Desktop/workspace_GEORGEBOT/src/MultipleLaserBot/compile_dependency/usr/local/lib/pkgconfig/
######################################c

echo "george test cmake command sh"
