import numpy as np

def calculate_iou(box1, box2):
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2

    inter_x1 = max(x1, x2)
    inter_y1 = max(y1, y2)
    inter_x2 = min(x1 + w1, x2 + w2)
    inter_y2 = min(y1 + h1, y2 + h2)

    inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)

    box1_area = w1 * h1
    box2_area = w2 * h2

    union_area = box1_area + box2_area - inter_area

    return inter_area / union_area

def calculate_precision(detections, groundtruths, iou_threshold=0.5):
    true_positives = 0
    for det in detections:
        for gt in groundtruths:
            if calculate_iou(det, gt) >= iou_threshold:
                true_positives += 1
                break
    
    total_predictions = len(detections)
    precision = true_positives / total_predictions if total_predictions > 0 else 0
    return precision

def calculate_recall(detections, groundtruths, iou_threshold=0.5):
    true_positives = 0
    for gt in groundtruths:
        for det in detections:
            if calculate_iou(det, gt) >= iou_threshold:
                true_positives += 1
                break
    
    total_actual_positives = len(groundtruths)
    recall = true_positives / total_actual_positives if total_actual_positives > 0 else 0
    return recall

def evaluate_performance(detections, groundtruth):
    total_iou = 0
    for det, gt in zip(detections, groundtruth):
        total_iou += calculate_iou(det, gt)
    average_iou = total_iou / len(detections)
    return average_iou
