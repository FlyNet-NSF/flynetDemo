#yum update
yum install -y yum-utils device-mapper-persistent-data lvm2 gcc zlib-devel openssl-devel firewalld
#yum remove python3

#add user
adduser -d /home/core -m core
echo 'core ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
#install docker
yum-config-manager --add-repo https://download.docker.com/linux/centos/docker-ce.repo
yum install -y docker-ce docker-ce-cli containerd.io
usermod -aG docker core
systemctl enable docker
systemctl restart docker

#install docker-compose
curl -L "https://github.com/docker/compose/releases/download/1.28.5/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
chmod +x /usr/local/bin/docker-compose

#install kubernetes
cat <<EOF > /etc/yum.repos.d/kubernetes.repo
[kubernetes]
name=Kubernetes
baseurl=https://packages.cloud.google.com/yum/repos/kubernetes-el7-x86_64
enabled=1
gpgcheck=1
repo_gpgcheck=1
gpgkey=https://packages.cloud.google.com/yum/doc/yum-key.gpg https://packages.cloud.google.com/yum/doc/rpm-package-key.gpg
EOF
yum install -y kubelet kubeadm kubectl
systemctl enable kubelet
systemctl start kubelet
hostnamectl set-hostname master-node

#set aliases
echo 192.168.125.10 master.flynetdemo.edu master-node >> /etc/hosts
echo 192.168.125.11 worker1.flynetdemo.edu worker-node1 >> /etc/hosts
echo 192.168.125.12 worker2.flynetdemo.edu worker-node2 >> /etc/hosts

#open firewall holes for kubernetes and rabbitmq
systemctl enable firewalld
systemctl start firewalld
firewall-cmd --permanent --add-port=2379-2380/tcp
firewall-cmd --permanent --add-port=4369/tcp
firewall-cmd --permanent --add-port=5671-5672/tcp
firewall-cmd --permanent --add-port=6443/tcp
firewall-cmd --permanent --add-port=8883/tcp
firewall-cmd --permanent --add-port=10000/tcp
firewall-cmd --permanent --add-port=10002/tcp
firewall-cmd --permanent --add-port=10250/tcp
firewall-cmd --permanent --add-port=10251/tcp
firewall-cmd --permanent --add-port=10252/tcp
firewall-cmd --permanent --add-port=10255/tcp
firewall-cmd --permanent --add-port=15672/tcp
firewall-cmd --permanent --add-port=25672/tcp
firewall-cmd --permanent --add-port=61613-61614/tcp

firewall-cmd --reload
cat <<EOF > /etc/sysctl.d/k8s.conf
net.bridge.bridge-nf-call-ip6tables = 1
net.bridge.bridge-nf-call-iptables = 1
EOF
sysctl --system

#disable selinux
setenforce permissive
sed -i 's/^SELINUX=enforcing$/SELINUX=permissive/' /etc/selinux/config

#unnecessary if getenforce==permissive, but at any rate...
setsebool -P nis_enabled 1

#disable swap (req'd for Kubernetes)
sed -i '/swap/d' /etc/fstab
eswapoff -a

#initialize kubernetes with flannel for now
/bin/su - core -c "sudo kubeadm init --pod-network-cidr=10.244.0.0/16"
#/bin/su - core -c "mkdir -p /home/core/.kube"
/bin/su - core -c "sudo cp -i /etc/kubernetes/admin.conf $HOME/.kube/config"
/bin/su - core -c "sudo chown $(id -u):$(id -g) $HOME/.kube/config"
/bin/su - core -c "sudo kubectl apply -f https://raw.githubusercontent.com/coreos/flannel/master/Documentation/kube-flannel.yml"

#install keadm
/bin/su - core -c "mkdir bin"
/bin/su - core -c "/usr/bin/wget https://github.com/kubeedge/kubeedge/releases/download/v1.6.0/keadm-v1.6.0-linux-amd64.tar.gz; tar -xzf keadm-v1.6.0-linux-amd64.tar.gz; ln -s /home/core/keadm-v1.6.0-linux-amd64/keadm/keadm /home/core/bin/"

/bin/su - core -c "/usr/bin/wget https://emmy8.casa.umass.edu/flynetDemo/core/docker-compose.yml"
/bin/su - core -c "/usr/bin/wget https://emmy8.casa.umass.edu/flynetDemo/core/rabbitmq.tar; /bin/tar -xf rabbitmq.tar"
/bin/su - core -c "/usr/local/bin/docker-compose up -d"
#/bin/su - core -c "/usr/bin/wget https://emmy8.casa.umass.edu/flynetDemo/core/talkToBasestation.tar; /bin/tar -xf talkToBasestation.tar"

