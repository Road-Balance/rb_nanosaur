sudo apt-get update && \
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y libglew-dev glew-utils libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev libglib2.0-dev \
    nvidia-tensorrt && \
    rm -rf /var/lib/apt/lists/*

cd /opt
sudo git clone https://github.com/dusty-nv/jetson-utils.git && \
sudo mkdir -p jetson-utils/build && cd jetson-utils/build && \
sudo cmake ../ && \
sudo make -j$(nproc) && \
sudo make install && \
sudo ldconfig

pip3 install wheel
pip3 install -U wstool

sudo apt-get update && \
    apt-get install libjpeg-dev zlib1g-dev python3-pip -y && \
    rm -rf /var/lib/apt/lists/*
