sudo systemctl start rai_robot.service
sudo systemctl restart rai_robot.service
sudo systemctl status rai_robot.service
sudo systemctl stop rai_robot.service

journalctl -u rai_robot.service -f