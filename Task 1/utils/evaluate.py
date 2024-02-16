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

# def calculate_precision(detections, groundtruths, iou_threshold=0.5):
#     """
#     Calculate precision based on IoU between detections and groundtruths
#     """
#     true_positives = 0
#     for det in detections:
#         for gt in groundtruths:
#             if calculate_iou(det, gt) >= iou_threshold:
#                 true_positives += 1
#                 break
    
#     total_predictions = len(detections)
#     precision = true_positives / total_predictions if total_predictions > 0 else 0
#     return precision

def calculate_distance(box1, box2):
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2

    center1 = np.array([x1 + w1 / 2, y1 + h1 / 2])
    center2 = np.array([x2 + w2 / 2, y2 + h2 / 2])

    return np.linalg.norm(center1 - center2)

def calculate_average_distance(detections, groundtruths):
    total_distance = 0
    for det, gt in zip(detections, groundtruths):
        total_distance += calculate_distance(det, gt)
    average_distance = total_distance / len(detections)
    return average_distance

def calculate_precision(detections, groundtruths, iou_threshold=20):
    """
    Calculate precision based on distance between centers of detections and groundtruths 
    """
    true_positives = 0
    for det in detections:
        for gt in groundtruths:
            det_center = np.array([det[0] + det[2] / 2, det[1] + det[3] / 2])
            gt_center = np.array([gt[0] + gt[2] / 2, gt[1] + gt[3] / 2])
            if np.linalg.norm(det_center - gt_center) <= iou_threshold:
                true_positives += 1
                break
    
    total_predictions = len(detections)
    precision = true_positives / total_predictions if total_predictions > 0 else 0
    return precision

# def calculate_recall(detections, groundtruths, iou_threshold=0.5):
#     """
#     Calculate reacall based on IoU between detections and groundtruths
#     """
#     true_positives = 0
#     for gt in groundtruths:
#         for det in detections:
#             if calculate_iou(det, gt) >= iou_threshold:
#                 true_positives += 1
#                 break
    
#     total_actual_positives = len(groundtruths)
#     recall = true_positives / total_actual_positives if total_actual_positives > 0 else 0
#     return recall

def calculate_recall(detections, groundtruths, iou_threshold=20):
    """
    Calculate reacall based on distance between centers of detections and groundtruths
    """
    true_positives = 0
    for gt in groundtruths:
        for det in detections:
            det_center = np.array([det[0] + det[2] / 2, det[1] + det[3] / 2])
            gt_center = np.array([gt[0] + gt[2] / 2, gt[1] + gt[3] / 2])
            if np.linalg.norm(det_center - gt_center) <= iou_threshold:
                true_positives += 1
                break
    
    total_actual_positives = len(groundtruths)
    recall = true_positives / total_actual_positives if total_actual_positives > 0 else 0
    return recall

def evaluate_average_iou(detections, groundtruth):
    total_iou = 0
    for det, gt in zip(detections, groundtruth):
        total_iou += calculate_iou(det, gt)
    average_iou = total_iou / len(detections)
    return average_iou

def visualize_iou(detections, groundtruth, title="IOU Across Frames"):
    import matplotlib.pyplot as plt

    frames = range(1, len(detections) + 1)
    total_iou = 0
    ious = []
    for det, gt in zip(detections, groundtruth):
        iou = calculate_iou(det, gt)
        ious.append(iou)
        total_iou += iou
    average_iou = total_iou / len(detections)
    plt.figure(figsize=(5, 5))
    plt.plot(frames, ious)
    plt.title(title)
    plt.xlabel("Frame Number")
    plt.ylabel("IOU")
    plt.grid()
    plt.axhline(y=average_iou, color='r', linestyle='--', label=f'Average IOU: {average_iou:.2f}')
    plt.show()

def visualize_iou_comparison(detections_tm, detections_tm_im, detections_kf, groundtruth, title="IOU Comparison Across Frames"):
    import matplotlib.pyplot as plt

    frames = range(1, len(detections_tm) + 1)
    total_iou_tm = 0
    total_iou_kf = 0
    total_iou_tm_im = 0
    ious_tm = []
    ious_kf = []
    ious_tm_im = []
    for det_tm, det_kf, det_tm_im, gt in zip(detections_tm, detections_kf, detections_tm_im, groundtruth):
        iou_tm = calculate_iou(det_tm, gt)
        iou_kf = calculate_iou(det_kf, gt)
        iou_tm_im = calculate_iou(det_tm_im, gt)
        ious_tm.append(iou_tm)
        ious_kf.append(iou_kf)
        ious_tm_im.append(iou_tm_im)
        total_iou_tm += iou_tm
        total_iou_kf += iou_kf
        total_iou_tm_im += iou_tm_im
    average_iou_tm = total_iou_tm / len(detections_tm)
    average_iou_kf = total_iou_kf / len(detections_kf)
    average_iou_tm_im = total_iou_tm_im / len(detections_tm_im)
    plt.figure(figsize=(5, 5))
    plt.plot(frames, ious_tm, label="TM")
    plt.plot(frames, ious_kf, label="KF")
    plt.plot(frames, ious_tm_im, label="TM_IM")
    plt.title(title)
    plt.xlabel("Frame Number")
    plt.ylabel("IOU")
    plt.grid()
    plt.axhline(y=average_iou_tm, color='r', linestyle='--', label=f'Success TM: {average_iou_tm:.2f}')
    plt.axhline(y=average_iou_kf, color='g', linestyle='--', label=f'Success KF: {average_iou_kf:.2f}')
    plt.axhline(y=average_iou_tm_im, color='b', linestyle='--', label=f'Success TM_IM: {average_iou_tm_im:.2f}')
    plt.legend()
    plt.show()

def visualize_precision_recall(detections, groundtruth, title="Precision-Recall Curve"):
    import matplotlib.pyplot as plt

    precisions = []
    recalls = []
    for i in range(1, len(detections) + 1):
        precision = calculate_precision(detections[:i], groundtruth)
        recall = calculate_recall(detections[:i], groundtruth)
        precisions.append(precision)
        recalls.append(recall)
    plt.figure(figsize=(5, 5))
    plt.plot(recalls, precisions)
    plt.title(title)
    plt.xlabel("Recall")
    plt.ylabel("Precision")
    plt.grid()
    plt.show()

def visualize_precision_recall_comparison(detections_tm, detections_tm_im, detections_kf, groundtruth, title="Precision-Recall Curve Comparison"):
    import matplotlib.pyplot as plt

    precisions_tm = []
    recalls_tm = []
    precisions_kf = []
    recalls_kf = []
    precisions_tm_im = []
    recalls_tm_im = []
    for i in range(1, len(detections_tm) + 1):
        precision_tm = calculate_precision(detections_tm[:i], groundtruth)
        recall_tm = calculate_recall(detections_tm[:i], groundtruth)
        precision_kf = calculate_precision(detections_kf[:i], groundtruth)
        recall_kf = calculate_recall(detections_kf[:i], groundtruth)
        precision_tm_im = calculate_precision(detections_tm_im[:i], groundtruth)
        recall_tm_im = calculate_recall(detections_tm_im[:i], groundtruth)
        precisions_tm.append(precision_tm)
        recalls_tm.append(recall_tm)
        precisions_kf.append(precision_kf)
        recalls_kf.append(recall_kf)
        precisions_tm_im.append(precision_tm_im)
        recalls_tm_im.append(recall_tm_im)
    plt.figure(figsize=(5, 5))
    plt.plot(recalls_tm, precisions_tm, label="TM")
    plt.plot(recalls_kf, precisions_kf, label="KF")
    plt.plot(recalls_tm_im, precisions_tm_im, label="TM_IM")
    plt.title(title)
    plt.xlabel("Recall")
    plt.ylabel("Precision")
    plt.grid()
    plt.legend()
    plt.show()

def visualize_distance(detections, groundtruth, title="Distance Across Frames"):
    import matplotlib.pyplot as plt

    frames = range(1, len(detections) + 1)
    distances = []
    for det, gt in zip(detections, groundtruth):
        distance = calculate_distance(det, gt)
        distances.append(distance)
    average_distance = np.mean(distances)
    plt.figure(figsize=(5, 5))
    plt.plot(frames, distances)
    plt.title(title)
    plt.xlabel("Frame Number")
    plt.ylabel("Distance")
    plt.axhline(y=average_distance, color='r', linestyle='--', label=f'Average Distance: {average_distance:.2f}')
    plt.grid()
    plt.show()

def visualize_distance_comparison(detections_tm, detections_tm_im, detections_kf, groundtruth, title="Distance Comparison Across Frames"):
    import matplotlib.pyplot as plt

    frames = range(1, len(detections_tm) + 1)
    distances_tm = []
    distances_kf = []
    distances_tm_im = []
    for det_tm, det_kf, det_tm_im, gt in zip(detections_tm, detections_kf, detections_tm_im, groundtruth):
        distance_tm = calculate_distance(det_tm, gt)
        distance_kf = calculate_distance(det_kf, gt)
        distance_tm_im = calculate_distance(det_tm_im, gt)
        distances_tm.append(distance_tm)
        distances_kf.append(distance_kf)
        distances_tm_im.append(distance_tm_im)
    plt.figure(figsize=(5, 5))
    plt.plot(frames, distances_tm, label="TM")
    plt.plot(frames, distances_kf, label="KF")
    plt.plot(frames, distances_tm_im, label="TM_IM")
    plt.title(title)
    plt.xlabel("Frame Number")
    plt.ylabel("Distance")
    plt.grid()
    plt.legend()
    plt.show()