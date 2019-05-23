make clean
sudo make
arm-linux-gnueabihf-strip libRockroboBridge.so
sshpass -p rockrobo scp libRockroboBridge.so root@192.168.8.1:/mnt/data/lib
