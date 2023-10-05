FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget zsh
RUN sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"


ENV WS=/workspaces
RUN mkdir -p $WS/src
RUN cd $WS/src && git clone https://github.com/introlab/rtabmap.git 
RUN apt install -y libsqlite3-dev libpcl-dev libopencv-dev git cmake libproj-dev libqt5svg5-dev
RUN add-apt-repository ppa:borglab/gtsam-develop \
   &&  apt install -y libgtsam-dev libgtsam-unstable-dev
RUN cd /tmp \ 
   && git clone https://github.com/RainerKuemmerle/g2o.git \ 
   && cd g2o \
   && mkdir build \
   && cd build \
   && cmake -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF .. \
   && make -j4 \
   && make install
RUN cd /$WS/src \
   && git clone --recursive https://github.com/luxonis/depthai-core.git \
   && cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local \
   && cmake --build depthai-core/build --target install --parallel 4 

RUN cd $WS/src/rtabmap/build && cmake -DWITH_DEPTHAI=ON .. && make -j4 && make install && ldconfig

CMD ["zsh"]