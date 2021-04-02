yum update
yum install -y yum-utils device-mapper-persistent-data lvm2 gcc zlib-devel openssl-devel
yum remove -y python3
adduser -d /home/drone -m drone
echo 'drone ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

cd /root
/usr/bin/wget https://www.python.org/ftp/python/3.7.9/Python-3.7.9.tgz
/usr/bin/tar -xzf Python-3.7.9.tgz
cd /root/Python-3.7.9
./configure 
make altinstall
/usr/local/bin/pip3.7 install pika
/usr/local/bin/pip3.7 install requests
/usr/local/bin/pip3.7 install geojson
/usr/local/bin/pip3.7 install geopy
/bin/su - drone -c "/usr/bin/wget https://emmy8.casa.umass.edu/flynetDemo/drone/listenToBasestation.tar; /bin/tar -xf listenToBasestation.tar"
/bin/su - drone -c "/usr/bin/wget https://emmy8.casa.umass.edu/flynetDemo/drone/sendToBasestation.tar; /bin/tar -xf sendToBasestation.tar"


