import os
import cv2
import numpy as np
import rospy
import yaml
import tensorflow as tf
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        
        self.classes = {1: TrafficLight.RED,
                        2: TrafficLight.YELLOW,
                        3: TrafficLight.GREEN,
                        4: TrafficLight.UNKNOWN}
        self.min_score_thresh = 0.5
        self.image_counter = 0
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.model_path = os.path.dirname(os.path.realpath(__file__)) + self.config['detection_model']
        self.detection_graph = self.load_graph(self.model_path)
        with tf.Session(graph=self.detection_graph) as sess:                
            self.sess = sess

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        img = self.preprocess_img(image)
        return self.predict(img, self.min_score_thresh)
    
    def preprocess_img(self, img):
        img = cv2.resize(img, (300, 300))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img
    
    def predict(self, image, min_score_thresh):
        ''' Below we load the graph and extract the relevant tensors using get_tensor_by_name. These tensors reflect the input and outputs of the graph, or least the ones we care about for detecting objects.'''

        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')

        # The classification of the object (integer id).
        detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        
        ''' Run detection and classification on an image.'''
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)

        # Actual detection.
        (boxes, scores, classes) = self.sess.run([detection_boxes, detection_scores, detection_classes], 
                                                 feed_dict={image_tensor: image_np})

        # Remove unnecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        count = {1: 0, 2: 0, 3: 0}  # 1 - Red, 2 - Yellow, 3 - Green
        for i in range(len(boxes)):
            if scores[i] > min_score_thresh:  
                light_state = self.classes[classes[i]]
                if light_state == TrafficLight.RED:
                    count[1] += 1
                elif light_state == TrafficLight.YELLOW:
                    count[2] += 1
                elif light_state == TrafficLight.GREEN:
                    count[3] += 1
                    
                    
        count_list = sorted(count.items(), key = lambda kv:(kv[1], kv[0])) # sort the count dictionary and then return the biggest one
        #rospy.logwarn("Number of Red: %s, Number of Yellow: %s, Number of Green: %s", count[1], count[2], count[3])
        if count_list[-1][1] != 0:
            return self.classes[count_list[-1][0]]
        else:
            return TrafficLight.UNKNOWN   # return UNKNOWN if not detect any of the traffic lights
        
    
    #
    # Utility funcs
    #        
    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)

        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes
    
    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].

        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width

        return box_coords
    
    def draw_boxes(self, image, boxes, classes, thickness=4):
        """Draw bounding boxes on the image"""
        color_list = {1: (255,0,0), 2: (255,255,0), 3: (0,128,0)}
        for i in range(len(boxes)):
            bot, left, top, right = boxes[i, ...]
            class_id = int(classes[i])
            cv2.rectangle(image, (left,top), (right,bot), color_list[class_id], thickness)
    