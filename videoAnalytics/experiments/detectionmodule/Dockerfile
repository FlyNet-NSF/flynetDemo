# Set CTO_NUMPROC to the number of processors on the server
ARG CTO_NUMPROC=12

from nvidia/cuda:10.1-cudnn7-devel
RUN apt-get update
RUN apt -y install build-essential checkinstall cmake pkg-config yasm unzip cuda-toolkit-10-1
RUN apt -y install libjpeg8-dev libpng-dev git gfortran software-properties-common nano
RUN apt -y update
RUN apt -y install libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
RUN apt -y install libxine2-dev libv4l-dev
RUN cd /usr/include/linux
RUN ln -s -f ../libv4l1-videodev.h videodev.h
RUN cd "$cwd"
RUN apt -y install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
RUN apt -y install libgtk2.0-dev libtbb-dev qt5-default libatlas-base-dev libfaac-dev libmp3lame-dev libtheora-dev libvorbis-dev libxvidcore-dev
RUN apt -y install libopencore-amrnb-dev libopencore-amrwb-dev libavresample-dev x264 v4l-utils
RUN apt -y install libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
RUN git clone https://github.com/opencv/opencv.git
WORKDIR opencv
RUN git checkout 3.4.6
WORKDIR ..
RUN git clone https://github.com/opencv/opencv_contrib.git
WORKDIR opencv_contrib
RUN git checkout 3.4.6
WORKDIR ..
WORKDIR opencv
RUN mkdir build
WORKDIR build
RUN cmake -DCMAKE_INSTALL_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local/ -DOPENCV_GENERATE_PKGCONFIG=ON -DWITH_WEBP=OFF -DINSTALL_C_EXAMPLES=OFF -DINSTALL_PYTHON_EXAMPLES=OFF -DBUILD_EXAMPLES=OFF -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DBUILD_DOCS=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DWITH_TBB=ON -D WITH_EIGEN=ON -D WITH_OPENMP=ON -DWITH_IPP=ON -DWITH_CSTRIPES=ON -DWITH_OPENCL=ON -DWITH_V4L=ON -DENABLE_FAST_MATH=1 -DFORCE_VTK=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DWITH_GTK=ON -DWITH_OPENMP=ON -DWITH_CUDA=ON -DCUDA_FAST_MATH=1 -DWITH_CUBLAS=1 -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10-1/ -DCUDA_cublas_LIBRARY=cublas -DCUDA_cufft_LIBRARY=cufft -DCUDA_nppim_LIBRARY=nppim -DCUDA_nppidei_LIBRARY=nppidei -DCUDA_nppif_LIBRARY=nppif -DCUDA_nppig_LIBRARY=nppig -DCUDA_nppim_LIBRARY=nppim -DCUDA_nppist_LIBRARY=nppist -DCUDA_nppisu_LIBRARY=nppisu -DCUDA_nppitc_LIBRARY=nppitc -DCUDA_npps_LIBRARY=npps -DCUDA_nppc_LIBRARY=nppc -DCUDA_nppial_LIBRARY=nppial -DCUDA_nppicc_LIBRARY=nppicc -D CUDA_nppicom_LIBRARY=nppicom ..
RUN make -j${CTO_NUMPROC} install
RUN rm -rf /opencv /opencv_contrib
RUN (apt-get autoremove -y; apt-get autoclean -y)
RUN mkdir /coconet
COPY . /coconet
WORKDIR /coconet
RUN curl -o yolov3.weights https://pjreddie.com/media/files/yolov3.weights
CMD ["./darknet" "detect" "cfg/yolov3.cfg" "yolov3.weights" "./dataset/"]