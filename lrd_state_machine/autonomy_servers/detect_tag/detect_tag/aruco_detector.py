# install opencv dependencies first:
# pip3 install opencv-python
# pip3 install opencv-contrib-python
from collections import namedtuple
import cv2
import numpy as np

###HYPERPARAMETERS######################################################
# length of markers we are detecting
MARKER_SIZE = 20.0 #14.7  # cm

# for detection of markers in subpixels
# we search for subpixels inside window and outside zero zone
WIN_SIZE = (5, 5)
ZERO_ZONE = (-1, -1)
# specifies desired accuracy, number of iterations, and termination
# criteria
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 40, 0.001)
# https://docs.opencv.org/4.x/d9/d5d/classcv_1_1TermCriteria.html

###HELPER FUNCTIONS#####################################################
# helper function to simplify cropping photo
def crop(image):
    #return image[0:image.shape[0], 0:image.shape[1]//2]
    return np.split(image, 2, axis=1)[0]

# helper function for printing text on screen
def print_text(image, text, pos):
    cv2.putText(
        image, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 
        0.5, (0, 255, 0), 1, cv2.LINE_AA
    )

###TAG DETECTOR#########################################################
# define object for storing tag corner data
# note: pos is normalized analog position on image, -1 is left, 0 is 
#       center, 1 is right
Tag = namedtuple(
    'Tag', ['id', 'tr', 'tl', 'br', 'bl', 'x_pos', 'y_pos', 'dist']
)


class Aruco_Detector():

    def __init__(self, calibration_data, data_folder, is_stereo=False):
        self.is_stereo = is_stereo

        # load calibration data from numpy file
        with np.load(f'{data_folder}/{calibration_data}') as X:
            self.mtx, self.dist, _ , _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

        # define aruco tagdictionary, tag detection parameters, 
        # and video capture feed for tag detection
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

    def detect_tags(self, frame):
        # create set to store ids for discovered tags, and list to hold 
        # tag data
        tag_ids = set()
        tags = []  
        
        if self.is_stereo: 
            frame = crop(frame)

        # detect markers in frame, get ids and corners of markers
        #corners, ids, _ = self.detector.detectMarkers(frame)
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self.dictionary, parameters=self.parameters
        )

        if corners:  # if there are corners detected
            # use a SubPixel corner refinement method to improve marker 
            # detection
            for corner in corners:
                cv2.cornerSubPix(
                    cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 
                    np.array(corner[0], dtype='float32'), 
                    WIN_SIZE, ZERO_ZONE, CRITERIA
                )

            # Pose refinement method to improve pose estimation
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, self.mtx, self.dist
            )

            for tag_id, corners, i in zip(ids, corners, range(0, ids.size)):
                # clean corner data and package as a single tuple
                clean_corners = corners.reshape(4, 2).astype(int) 
                
                tr_pos = clean_corners[0].ravel()
                tl_pos = clean_corners[1].ravel()
                br_pos = clean_corners[2].ravel()

                tag_center_x = (tr_pos[0]+tl_pos[0]) / 2
                tag_center_y = (tr_pos[1]+br_pos[1]) / 2

                normalized_tag_x = (tag_center_x-frame.shape[1]/2) / (frame.shape[1]/2)
                normalized_tag_y = (tag_center_y-frame.shape[0]/2) / (frame.shape[0]/2)
                
                tag = Tag(
                    tag_id[0],
                    clean_corners[0].ravel(),
                    clean_corners[1].ravel(),
                    clean_corners[2].ravel(),
                    clean_corners[3].ravel(),
                    normalized_tag_x,
                    normalized_tag_y,
                    round(tvecs[i][0][2],2)
                )

                # add tag to dataset of tags if not already present
                if tag.id not in tag_ids:
                    tag_ids.add(tag.id)
                    tags.append(tag)

                # draw vector lines and data to frame 
                cv2.polylines(  # create vector lines to draw
                    frame, [corners.astype(np.int32)], True,
                    (255, 0, 0), 4, cv2.LINE_4
                )
                # draw position vector, position in frame, and distance 
                # from camera
                cv2.drawFrameAxes(
                    frame, self.mtx, self.dist, rvecs[i], tvecs[i], 4, 4
                )
                
                print_text(frame, f'id: {tag.id} Dist: {tag.dist}', tag.tr)
                print_text(frame, f'Pos: ({tag.x_pos:.2}, {tag.y_pos:.2})', tag.br)

        return tags, frame  
        
    def __del__(self):
        # release all resources
        self.cap.release()
        cv2.destroyAllWindows()