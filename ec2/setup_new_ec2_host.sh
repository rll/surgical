# Edit the hostname 
# sudo hostname surgical-ec2
# sudo vim /etc/hostname change to surgical-ec2
# sudo vim /etc/hosts add 127.0.1.1 surgical-ec2


sudo apt-get update
sudo apt-get install xserver-xorg xserver-xorg-core openbox
sudo apt-get install g++ gcc libeigen2-dev libgle3 libgle3-dev fortran-compiler libglut3 libglut
sudo apt-get install subversion git-core

#ssh -i sameep-surgical.pem ubuntu@ec2-50-16-35-135.compute-1.amazonaws.com 'cd /home/ubuntu/code/trunk/surgical; svn up --config-option config:tunnels:ssh="ssh -a -i /home/ubuntu/.ssh/id_rsa_st"'

#cd /home/ubuntu/code/trunk/surgical; rm -rf *; svn up --config-option config:tunnels:ssh="ssh -a -i /home/ubuntu/.ssh/id_rsa_st"
#cd /home/ubuntu/code/trunk/surgical/DiscreteRods; make clean; make clean; cd LearnParams; make clean; make clean; cd ..; make clean; make; cd LearnParams; make clean; make;
