# This script installs and configures OpenCV 3 and Python 3
# This installation uses the virtual environments and after installation one should always run the following two commands to make sure everything works correctly:
#source ~/.profile	# to initialize potential changes
#workon cv		# create virtual environment cv
#python			# start using python

# install guide: http://www.pyimagesearch.com/2016/04/18/install-guide-raspberry-pi-3-raspbian-jessie-opencv-3/

versioncv=3.1.0

# Update Ubuntu and install the needed dependencies
echo "Updating Ubuntu"
sudo apt-get update

echo "Installing some developer tools (cmake etc.)"
sudo apt-get install build-essential cmake pkg-config

echo "Installing some image I/O packages: to load various image file formats from disk"
sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev

echo "Installing some video I/O packages: to read various video file formats from disk as well as work directly with video streams"
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev

echo "Installing the GTK development library: used to compile highgui (sub-module of OpenCV library used to display images and build basic GUIs)"
sudo apt-get install libgtk2.0-dev

echo "Optimize many operations inside of OpenCV (namely matrix operations) by installing a few extra dependencies"
sudo apt-get install libatlas-base-dev gfortran

echo "Installing Python 2.7 and Python 3 header files: to compile OpenCV with Python bindings"
sudo apt-get install python2.7-dev python3-dev

# Install OpenCV
echo "Downloading OpenCV" $versioncv "to home folder"
cd ~
mkdir opencv_install_files
cd opencv_install_files
wget -O opencv-$versioncv.zip https://github.com/Itseez/opencv/archive/$versioncv.zip
unzip opencv-$versioncv.zip

echo "Downloading opvencv_contrib: to have the full installation of OpenCV"
wget -O opencv_contrib-$versioncv.zip https://github.com/Itseez/opencv_contrib/archive/$versioncv.zip
unzip opencv_contrib-$versioncv.zip

echo "Installing pip: a Python package manager"
wget https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py

echo "Installing virtualenv and virtualenvwrapper: for good practice while coding Python"
sudo pip install virtualenv virtualenvwrapper
sudo rm -rf ~/.cache/pip
echo -e "\n# virtualenv and virtualenvwrapper" >> ~/.profile
echo "export WORKON_HOME=$HOME/.virtualenvs" >> ~/.profile
echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.profile
source ~/.profile 	#update system variables

echo "Creating Python virtual environment: need to be in here everytime from now on! Check if (cv) is preceding the prompt!"
mkvirtualenv cv -p python3

echo "Installing NumPy"
pip install numpy

echo "Compiling(installing) OpenCV" $versioncv
cd opencv-$versioncv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_install_files/opencv_contrib-$versioncv/modules \
    -D BUILD_EXAMPLES=ON ..
make -j4
sudo make install
sudo ldconfig

# Additional configurations
echo "Some additional configurations"
ls -l /usr/local/lib/python3.5/site-packages/
cd /usr/local/lib/python3.5/site-packages/
sudo mv cv2.cpython-35m-x86_64-linux-gnu.so cv2.so
cd ~/.virtualenvs/cv/lib/python3.5/site-packages/
ln -s /usr/local/lib/python3.5/site-packages/cv2.so cv2.so

echo "OpenCV" $versioncv "ready to be used"
echo 'Reboot your computer'
