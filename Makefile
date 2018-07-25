b:
	platformio run
u:
	platformio run -t upload
m:
	platformio device monitor --port /dev/ttyACM0 --baud 115200
