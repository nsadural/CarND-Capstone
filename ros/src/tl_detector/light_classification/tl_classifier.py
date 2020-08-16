import os
import cv2
import numpy as np
import rospy
import yaml
import tensorflow as tf
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        
        self.classes = {1: TrafficLight.RED,
                        2: TrafficLight.YELLOW,
                        3: TrafficLight.GREEN,
                        4: TrafficLight.UNKNOWN}
        self.min_score_thresh = 0.5
        self.image_counter = 0
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.model_path = os.path.dirname(os.path.realpath(__file__)) + self.config['detection_model']

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        img = self.preprocess_img(image)
        return self.predict(img, self.min_score_thresh)
    
    def preprocess_img(self, img):
#         img = cv2.resize(img, (300, 300))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img
    
    def predict(self, image, min_score_thresh):
        ''' Below we load the graph and extract the relevant tensors using get_tensor_by_name. These tensors reflect the input and outputs of the graph, or least the ones we care about for detecting objects.'''
        detection_graph = self.load_graph(self.model_path)

        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')

        # The classification of the object (integer id).
        detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        
        ''' Run detection and classification on an image.'''
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)

        with tf.Session(graph=detection_graph) as sess:                
            # Actual detection.
            (boxes, scores, classes) = sess.run([detection_boxes, detection_scores, detection_classes], 
                                                feed_dict={image_tensor: image_np})

            # Remove unnecessary dimensions
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)
            
            count_NonRed = count_red = 0
            for i in range(len(boxes)):
                if scores[i] > min_score_thresh:  
                    light_state = self.classes[classes[i]]
                    if light_state == TrafficLight.RED:
                        count_red += 1
                    else:
                        count_NonRed += 1
            
            

            # Uncomment to see the visulized result of the classification with box drawing
            confidence_cutoff = 0.8
            # Filter boxes with a confidence score less than `confidence_cutoff`
            boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)

            # The current box coordinates are normalized to a range between 0 and 1.
            # This converts the coordinates actual location on the image.
            width, height = image.shape[1], image.shape[0]
            box_coords = self.to_image_coords(boxes, height, width)

            # Each class with be represented by a differently colored box
            self.draw_boxes(image, box_coords, classes)
            
#             plt.figure(figsize=(12, 8))
#             plt.imshow(image)

            # Save the image if detect the traffic light is on
            cv2.imwrite(os.path.join('/saved_images/', "image_%04i.jpg" % (self.image_counter)), image)
            self.image_counter += 1

            # if the number of red traffic light is larger than the one of non-red lights, return RED state
            if count_red < count_NonRed:
                return TrafficLight.GREEN
            elif count_red != 0 and count_red >= count_NonRed:
                rospy.logwarn("RED Traffic Light detected by the model!")
                return TrafficLight.RED
            else:
                return TrafficLight.UNKNOWN

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
    