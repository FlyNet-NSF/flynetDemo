yum update
yum install -y yum-utils device-mapper-persistent-data lvm2 gcc zlib-devel openssl-devel make gcc
yum remove -y python3
yum-config-manager --add-repo https://download.docker.com/linux/centos/docker-ce.repo
yum install -y docker-ce docker-ce-cli containerd.io
groupadd docker
adduser -d /home/basestation -m basestation
usermod -aG docker basestation
systemctl enable docker
systemctl restart docker
curl -L "https://github.com/docker/compose/releases/download/1.28.5/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
chmod +x /usr/local/bin/docker-compose

cd /root
/usr/bin/wget https://www.python.org/ftp/python/3.7.9/Python-3.7.9.tgz
/usr/bin/tar -xzf Python-3.7.9.tgz
cd /root/Python-3.7.9
./configure 
make altinstall
python3.7 -m pip install --upgrade pip
/usr/local/bin/pip3.7 install python-hosts
/usr/local/bin/pip3.7 install pika
/usr/local/bin/pip3.7 install requests
/usr/local/bin/pip3.7 install geojson
/usr/local/bin/pip3.7 install geopy
/bin/su - basestation -c "/usr/bin/wget https://emmy8.casa.umass.edu/flynetDemo/basestation/docker-compose.yml"
/bin/su - basestation -c "/usr/bin/wget https://emmy8.casa.umass.edu/flynetDemo/basestation/rabbitmq.tar; /bin/tar -xf rabbitmq.tar"
/bin/su - basestation -c "/usr/local/bin/docker-compose up -d"
/bin/su - basestation -c "/usr/bin/wget https://emmy8.casa.umass.edu/flynetDemo/basestation/listenToDrone.tar; /bin/tar -xf listenToDrone.tar"

