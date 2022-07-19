import sys
import time
import cv2
import numpy as np

def main(src,shape=[640, 480]):
	new_src = "/dev/video" + str(src)
	camera = cv2.VideoCapture(new_src)
	# camera.set(cv2.CAP_PROP_FRAME_HEIGHT, shape[0])
	# camera.set(cv2.CAP_PROP_FRAME_WIDTH, shape[1])
	# While Balance
	start_time = time.time()
	while time.time() - start_time < 3:
		camera.read()
	# for i in range(10):
	return_value, image = camera.read()
	cv2.imwrite('{}.jpg'.format(src), image)
	del(camera)


if __name__ == "__main__":
	src = 0;
	if len(sys.argv) > 1:
		src = int(sys.argv[1]);
	print(src)
	main(src);
