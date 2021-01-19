
import numpy as np 
import cv2

cap = cv2.VideoCapture(0)

# cap.set(4,1920)
# cap.set(5,1080)

while(True):
	ret,frame =cap.read()

	# image= cv2.cvtColor(frame)

	cv2.imshow('frame', frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows() 