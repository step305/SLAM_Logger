rm ~/SLAM_FIFO.tmp

echo "2-3.2" | sudo tee /sys/bus/usb/drivers/usb/unbind
echo "2-3.2" | sudo tee /sys/bus/usb/drivers/usb/bind
echo "1-2.3" | sudo tee /sys/bus/usb/drivers/usb/unbind
echo "1-2.3" | sudo tee /sys/bus/usb/drivers/usb/bind