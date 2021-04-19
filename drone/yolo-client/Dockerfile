FROM centos:8

RUN yum install -y yum-utils gcc zlib-devel mesa-libGL-devel wget make openssl-devel
RUN yum remove -y python3
RUN cd /root && /usr/bin/wget https://www.python.org/ftp/python/3.7.9/Python-3.7.9.tgz && /usr/bin/tar -xzf Python-3.7.9.tgz && cd Python-3.7.9 && ./configure && make install
RUN pip3.7 install requests
RUN pip3.7 install opencv-python
RUN mkdir -p /opt/video_client

WORKDIR /opt/video_client
VOLUME ["/opt/video_client"]
COPY video_client.py /opt/video_client/
COPY requirements.txt /opt/video_client/
RUN pip3.7 install --no-cache-dir -r requirements.txt

ENV SERVER=192.168.135.12:5000
COPY docker-entrypoint.sh /opt/video_client/
CMD ["/bin/sh", "./docker-entrypoint.sh"]
