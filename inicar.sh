cd /usr/local/webots
bash webots&
sleep 2
bash /home/robocomp/robocomp/tools/rcnode/rcnode.sh&
/home/usuario/robocomp/components/webots-bridge/bin/Webots2Robocomp /home/usuario/robocomp/components/webots-bridge/etc/config&
/home/usuario/robocomp/components/robocomp-robolab/components/hardware/laser/lidar3D/bin/Lidar3D  /home/usuario/robocomp/components/robocomp-robolab/components/hardware/laser/lidar3D/etc/config_pearl_webots&
sleep 5
~/robocomp/components/robotica_grupo7/chocachoca/bin/chocachoca ~/robocomp/components/robotica_grupo7/chocachoca/etc/config
