import cv2
import numpy as np
from typing import NamedTuple


class Resolution(NamedTuple):
    width: int = 4416
    height: int = 1242


class ZedCamera:

    def __init__(self, camera=0):
        self.cap = cv2.VideoCapture(camera)
        print('camera connected')

        image_size = Resolution(2208, 1242)    

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width * 2)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)  

    def read(self):
        ret, frame = self.cap.read()
        frame = np.split(frame, 2, axis=1)[0]
        frame = cv2.resize(frame, (2208, 1242), interpolation=cv2.INTER_AREA)

        return ret, frame
    
# def main():
#     zed = ZedCamera(2)
#     while True:
#         ret, frame = zed.read()
#         cv2.imshow('frame', frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()