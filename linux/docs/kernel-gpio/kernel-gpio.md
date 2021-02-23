# Kernel GPIO

	查看GPIO使用情况:
	cat /sys/kernel/debug/gpio

	查看是否存在对应GPIO文件:
	cat /sys/class/gpio/gpio%d

	使能GPIO引脚:
	echo n > /sys/class/gpio/export

	设置方向:
	echo \"out\" > /sys/class/gpio/gpio%d/direction\"
	echo \"in\" > /sys/class/gpio/gpio%d/direction\"

	设置高或低电平:
	echo 0 > /sys/class/gpio/gpio%d/value
	echo 1 > /sys/class/gpio/gpio%d/value
