# -*-Makefile-*-


ROS2_PLUGIN_DIR = /etc/fos/plugins/plugin-fdu-ros2
PLUGIN_CONF = $(ROS2_PLUGIN_DIR)/ros2_plugin.json
SYSTEMD_DIR = /lib/systemd/system/
BIN_DIR = /usr/bin
UUID = $(shell ./to_uuid.sh)

all:
	mkdir -p utils
	gcc -o utils/isolate isolate.c


clean:
	rm -rf utils/isolate

install:
	sudo pip3 install jinja2 psutil
ifeq "$(wildcard $(ROS2_PLUGIN_DIR))" ""
	mkdir -p $(ROS2_PLUGIN_DIR)
	sudo cp -r ./templates $(ROS2_PLUGIN_DIR)
	sudo cp -r ./utils $(ROS2_PLUGIN_DIR)
	sudo cp ./ros2_plugin $(ROS2_PLUGIN_DIR)
	sudo cp ./ROS2FDU.py $(ROS2_PLUGIN_DIR)
	sudo cp ./README.md $(ROS2_PLUGIN_DIR)
	sudo cp ./ros2_plugin.json $(PLUGIN_CONF)
else
	sudo cp -r ./templates $(ROS2_PLUGIN_DIR)
	sudo cp -r ./utils $(ROS2_PLUGIN_DIR)
	sudo cp ./ros2_plugin $(ROS2_PLUGIN_DIR)
	sudo cp ./ROS2FDU.py $(ROS2_PLUGIN_DIR)
	sudo cp ./README.md $(ROS2_PLUGIN_DIR)
endif
	sudo ln -sf $(ROS2_PLUGIN_DIR)/utils/isolate $(BIN_DIR)/fos_ros_isolate
	sudo cp ./fos_ros2.service $(SYSTEMD_DIR)
	sudo sh -c "echo $(UUID) | xargs -i  jq  '.configuration.nodeid = \"{}\"' $(PLUGIN_CONF) > /tmp/ros2_plugin.tmp && mv /tmp/ros2_plugin.tmp $(PLUGIN_CONF)"

uninstall:
	sudo systemctl disable fos_ros2
	sudo rm -rf $(ROS2_PLUGIN_DIR)
	sudo rm -rf /var/fos/ros2
	sudo rm $(SYSTEMD_DIR)/fos_ros2.service
