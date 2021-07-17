#!/usr/bin/env python3

import argparse
from pprint import pprint
from pathlib import Path
from collections import defaultdict
import matplotlib.pyplot as plt 
import numpy as np

def get_tpr(iou_lst, threshold):
    tp = sum(iou_lst >= threshold)
    TPR = tp / len(iou_lst)
    return TPR 

def get_auc(iou_lst):
    auc_curve = defaultdict(list)
    threshold_range = np.linspace(0, 1, num = 100)
    for threshold in threshold_range:
        TPR = get_tpr(iou_lst, threshold)
        auc_curve["threshold"].append(threshold)
        auc_curve["TPR"].append(TPR)
    s = 0
    for i in range(len(threshold_range) - 1):
        s += (
            (auc_curve["threshold"][i+1] - auc_curve["threshold"][i]) 
            *
            auc_curve["TPR"][i]
        )
    auc_curve["AUC"] = s
    return auc_curve

def get_precision(tp, fp):
    return tp / (tp + fp)

def get_IoU(target_box, pred_box):
    boxA = [
            target_box[0],
            target_box[1],
            target_box[0] + target_box[2],
            target_box[1] + target_box[3]
    ]
    boxB = [
            pred_box[0],
            pred_box[1],
            pred_box[0] + pred_box[2],
            pred_box[1] + pred_box[3]
    ]
 
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    # compute the area of intersection rectangle
    interArea = abs(max((xB - xA, 0)) * max((yB - yA), 0))
    if interArea == 0:
        return 0
    # compute the area of both the prediction and ground-truth
    # rectangles
    boxAArea = abs((boxA[2] - boxA[0]) * (boxA[3] - boxA[1]))
    boxBArea = abs((boxB[2] - boxB[0]) * (boxB[3] - boxB[1]))

    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = interArea / float(boxAArea + boxBArea - interArea)

    # return the intersection over union value
    return iou


    
def threshold_type(s):
    try:
        threshold = float(s)
        if threshold < 0 or threshold > 1:
            raise ValueError()
    except ValueError:
        raise argparse.ArgumentTypeError(f"threshold maybe from [0, 1]: '{s}'")
    return threshold  

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Get object tracking metric.')
    parser.add_argument(
        'target_file', type=Path,
        help='file with target boxes',
    )
    parser.add_argument(
        'prediction_file', type=Path,
        help='file with prediction boxes',
    )
    parser.add_argument(
        '-t', '--threshold', type=threshold_type,
        default=0.75,
        help='IoU threshold'
    )
    parser.add_argument(
        '-p', '--plot', type=bool,
        default=False,
        help='Plot AUC'
    )
    args = parser.parse_args()

    target_boxes = []
    with open(args.target_file, 'r') as f:
        for line in f.readlines():
            target_boxes.append([float(x) for x in line.split()])

    pred_boxes = []
    with open(args.prediction_file, 'r') as f:
        for line in f.readlines():
            pred_boxes.append([float(x) for x in line.split()])

    tp = 0
    fp = 0
    iou_lst = []
    for target_box, pred_box in zip(target_boxes, pred_boxes):
        iou = get_IoU(target_box, pred_box)
        if iou >= args.threshold:
            tp += 1
        else:
            fp += 1
        iou_lst.append(iou)
    iou_lst = np.array(iou_lst)
    mean_iou = np.mean(iou_lst)

    auc_curve = dict(get_auc(iou_lst))

    print("Precision: {:.3f}".format(get_precision(tp, fp)))
    print("AUC: {:.3f}".format(auc_curve["AUC"]))
    print("SR0.5: {:.3f}".format(
        get_tpr(iou_lst, 0.5)
    ))
    print("SR0.75: {:.3f}".format(
        get_tpr(iou_lst, 0.75)
    ))
    print("Mean IoU: {:.3f}".format(mean_iou))

    if args.plot:
        plt.plot(auc_curve["threshold"], auc_curve["TPR"])
        plt.xlabel("Threshold")
        plt.ylabel("TPR")
        plt.show()
