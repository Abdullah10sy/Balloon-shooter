"""
YOLOv5 Balloon Detection Module
Handles object detection using a custom-trained YOLOv5 model for balloon detection.
"""

import torch
import cv2
import numpy as np
from pathlib import Path


class BalloonDetector:
    def __init__(self, model_path='yolov5s.pt', confidence_threshold=0.5):
        """
        Initialize the YOLOv5 balloon detector.
        
        Args:
            model_path (str): Path to the YOLOv5 model file
            confidence_threshold (float): Minimum confidence for detections
        """
        self.confidence_threshold = confidence_threshold
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # Load YOLOv5 model
        try:
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', 
                                      path=model_path, force_reload=True)
            self.model.to(self.device)
            self.model.conf = confidence_threshold
            print(f"Model loaded successfully on {self.device}")
        except Exception as e:
            print(f"Error loading model: {e}")
            # Fallback to pretrained model for testing
            self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
            self.model.to(self.device)
            print("Using pretrained YOLOv5s model as fallback")
    
    def detect_balloons(self, frame):
        """
        Detect balloons in the given frame.
        
        Args:
            frame (numpy.ndarray): Input image frame
            
        Returns:
            list: List of balloon detections with format [x1, y1, x2, y2, confidence, class]
        """
        # Run inference
        results = self.model(frame)
        
        # Extract detections
        detections = results.pandas().xyxy[0].values
        
        # Filter for balloon class (assuming class 0 is balloon in custom model)
        # For pretrained model, we'll use 'sports ball' class (32) as proxy
        balloon_detections = []
        
        for detection in detections:
            x1, y1, x2, y2, conf, cls, name = detection
            
            # Filter by class name (adjust based on your custom model)
            if name.lower() in ['balloon', 'sports ball'] and conf >= self.confidence_threshold:
                balloon_detections.append([int(x1), int(y1), int(x2), int(y2), conf, int(cls)])
        
        return balloon_detections
    
    def get_balloon_center(self, detection):
        """
        Calculate the center point of a balloon detection.
        
        Args:
            detection (list): Balloon detection [x1, y1, x2, y2, conf, cls]
            
        Returns:
            tuple: (center_x, center_y)
        """
        x1, y1, x2, y2 = detection[:4]
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        return center_x, center_y
    
    def draw_detections(self, frame, detections):
        """
        Draw bounding boxes and labels on the frame.
        
        Args:
            frame (numpy.ndarray): Input image frame
            detections (list): List of balloon detections
            
        Returns:
            numpy.ndarray: Frame with drawn detections
        """
        annotated_frame = frame.copy()
        
        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw center point
            center_x, center_y = self.get_balloon_center(detection)
            cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # Draw label
            label = f'Balloon: {conf:.2f}'
            cv2.putText(annotated_frame, label, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return annotated_frame
    
    def get_frame_center(self, frame):
        """
        Get the center coordinates of the frame.
        
        Args:
            frame (numpy.ndarray): Input image frame
            
        Returns:
            tuple: (center_x, center_y)
        """
        height, width = frame.shape[:2]
        return width // 2, height // 2