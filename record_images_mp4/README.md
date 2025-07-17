docker run -it --network=host --volume /home/carma/gwmp_data:/gwmp_data usdotfhwastol/carma-arena-camera-driver:data-collection-3.8.0 bash
source /opt/carma/install/setup.bash
source /gwmp_data/kinetic_ws/devel/setup.bash
roslaunch record_images_mp4 all_cameras.launch
roslaunch record_images_mp4 record_six_cameras.launch

to edit exposure and other params: nano /opt/carma/install/arena_camera/share/arena_camera/launch/mono_camera.launch
