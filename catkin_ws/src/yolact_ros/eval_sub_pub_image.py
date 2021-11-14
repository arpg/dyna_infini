#!/usr/bin/env python3

from data import COCODetection, get_label_map, MEANS, COLORS
from yolact import Yolact
from utils.augmentations import BaseTransform, FastBaseTransform, Resize
from utils.functions import MovingAverage, ProgressBar
from layers.box_utils import jaccard, center_size, mask_iou
from utils import timer
from utils.functions import SavePath
from layers.output_utils import postprocess, undo_image_transformation
import pycocotools

from data import cfg, set_cfg, set_dataset

import numpy as np
import torch
import torch.backends.cudnn as cudnn
from torch.autograd import Variable
import argparse
import time
import random
import cProfile
import pickle
import json
import os
from collections import defaultdict
from pathlib import Path
from collections import OrderedDict
from PIL import Image

import matplotlib.pyplot as plt
import cv2

from multiprocessing.pool import ThreadPool
from queue import Queue

import rospy
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg

class Params:
    def __init__(self):
        self.trained_model = ""
        self.top_k = 15
        self.cuda = True
        self.fast_nms = True
        self.cross_class_nms = False
        self.display_masks = True
        self.display = True
        self.config = None
        self.display_lincomb = False
        self.image = None
        self.score_threshold = 0.15
        self.dataset = None
       
        #Normally wont change
        self.no_bar = False
        self.resume=False
        self.output_coco_json=False
        self.output_web_json=False
        self.benchmark=False
        self.no_sort=False
        self.no_hash=False
        self.mask_proto_debug=False
        self.crop=True
        self.detect = False

class ProcessImage:
    def __init__(self, args):
        model_path = SavePath.from_str(args.trained_model)
        args.config = model_path.model_name + '_config'
        print('Config Parsed %s from the file name.\n' % args.config)
        set_cfg(args.config)

        if args.cuda:
            cudnn.fastest = True
            torch.set_default_tensor_type('torch.cuda.FloatTensor')
        else:
            torch.set_default_tensor_type('torch.FloatTensor')

        with torch.no_grad():

            cudnn.fastest = True
            torch.set_default_tensor_type('torch.cuda.FloatTensor')

            dataset = None        

            print('Loading model...', end='')
            self.yolact_net = Yolact()
            self.yolact_net.load_weights(args.trained_model)
            self.yolact_net.eval()
            print(' Done.')

            if args.cuda:
                self.yolact_net = self.yolact_net.cuda()

        sub_topic = rospy.get_param("sub_topic", "/gray_image")
        pub_topic = rospy.get_param("pub_topic", "/image_seg")
        self.image_sub = rospy.Subscriber(sub_topic,numpy_msg(Image),self.ImageCallback)
        self.image_pub = rospy.Publisher(pub_topic,Image, queue_size=100)

        self.detection = rospy.get_param("detect_objects", "all")

        self.process_one_image = True
        self.imq = Queue()
        self.timeq = Queue()

    def ImageCallback(self,data):
        #print("receive images....... ")
        
        im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width,-1)
        self.imq.put(im)
        self.timeq.put(data.header.stamp)

        if self.process_one_image == True:
            im_front = self.imq.get()
            timestamp = self.timeq.get()
            self.process_one_image = False
            self.evaluate(im_front, timestamp)


    def evaluate(self, data, timestamp, train_mode=False):
        self.yolact_net.detect.use_fast_nms = args.fast_nms
        self.yolact_net.detect.use_cross_class_nms = args.cross_class_nms
        cfg.mask_proto_debug = args.mask_proto_debug

        self.evalimage(data, timestamp)

    def evalimage(self, data, timestamp, save_path:str=None):
        frame = torch.from_numpy(data).cuda().float()
        batch = FastBaseTransform()(frame.unsqueeze(0))

        #if we just want to predict one image ,it costs longer because the GPU needs time to initialize.
        #RTX3090 for image 640x480, generally use 12ms but if we just predict one, it uses up to 200ms
        preds = self.yolact_net(batch)

        img_numpy = self.prep_display(preds, frame, None, None, undo_transform=False)

        h,w,_ = frame.shape

        msg = Image()
        msg.header.stamp = timestamp
        msg.height = h
        msg.width = w
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = w
        msg.data = img_numpy.tobytes()

        self.image_pub.publish(msg)

        self.process_one_image = True

    def prep_display(self, dets_out, img, h, w, undo_transform=True, class_color=False, mask_alpha=0.45, fps_str=''):
        """
        Note: If undo_transform=False then im_h and im_w are allowed to be None.
        """
        if undo_transform:
            img_numpy = undo_image_transformation(img, w, h)
            img_gpu = torch.Tensor(img_numpy).cuda()
        else:
            img_gpu = img / 255.0
            h, w, _ = img.shape
        
        with timer.env('Postprocess'):
            save = cfg.rescore_bbox
            cfg.rescore_bbox = True
            t = postprocess(dets_out, w, h, visualize_lincomb = args.display_lincomb,
                                            crop_masks        = args.crop,
                                            score_threshold   = args.score_threshold)
            cfg.rescore_bbox = save

        with timer.env('Copy'):
            idx = t[1].argsort(0, descending=True)[:args.top_k]
            
            if cfg.eval_mask_branch:
                # Masks are drawn on the GPU, so don't copy
                masks = t[3][idx]

            #if classes[j] = 0 then it is a person
            classes, scores, boxes = [x[idx].cpu().detach().numpy() for x in t[:3]]

        num_dets_to_consider = min(args.top_k, classes.shape[0])
        for j in range(num_dets_to_consider):
            if scores[j] < args.score_threshold:
                num_dets_to_consider = j
                break


        # First, draw the masks on the GPU where we can do it really fast
        # Beware: very fast but possibly unintelligible mask-drawing code ahead
        # I wish I had access to OpenGL or Vulkan but alas, I guess Pytorch tensor operations will have to suffice
        if args.display_masks and cfg.eval_mask_branch and num_dets_to_consider > 0:
            indices_persons = []

            if self.detection == "persons":
                for j in range(num_dets_to_consider):
                    if classes[j] == 0:
                        indices_persons.append(j)
                masks = masks[indices_persons,:,:,None]
            elif self.detection == "all": 
                masks = masks[:num_dets_to_consider,:,:,None]
            else:
                masks = masks[:num_dets_to_consider,:,:,None]

            # sum(sim=0) will add all other dimensions together and eliminate the first dimension.
            # For example, original tensor torch.Size([15, 640, 480, 1])
            # After sum(dim=0) we have torch.Size([640, 480, 1])
            # Then after sum(dim=2) we have torch.Size([640, 480])
            #This means All the masks will be in one tensor
            masks = masks.sum(dim=0).sum(dim=2)
            #NOTE: A pixel that has two masks will get 2, we need to make sure every mask pixel is 1 so that when it times 255 we have 255
            masks[masks!=0] = 1


        # Then draw the stuff that needs to be done on the cpu
        # Note, make sure this is a uint8 tensor or opencv will not anti alias text for whatever reason
        img_numpy = (masks * 255).byte().cpu().numpy()
        
        return img_numpy

if __name__ == '__main__':

    rospy.init_node('yolact_detector', anonymous=True)

    args = Params()
    trained_model = rospy.get_param("trained_model")
    top_k = int(rospy.get_param("top_k", 15))
    score_threshold = rospy.get_param("score_threshold", 0.12)

    args.trained_model = trained_model
    args.top_k = top_k
    args.score_threshold = score_threshold

    pi = ProcessImage(args)

    rospy.spin()


