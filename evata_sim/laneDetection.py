import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import time
from pathlib import Path
import torch
from evata_sim.utils.utils import \
    time_synchronized, select_device, increment_path, \
    scale_coords, xyxy2xywh, non_max_suppression, split_for_trace_model, \
    driving_area_mask, lane_line_mask, plot_one_box, show_seg_result, \
    AverageMeter, LoadImages

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    #print(sem_img.shape)
    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
     
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    
    return img, ratio, (dw, dh)

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        src_dir = dir_path.split('/install')[0]  # install kısmını çıkar
        weights = os.path.join(src_dir, 'src', 'evata_sim', 'evata_sim', 'utils', 'yolopv2.pt')
        device = "0"
        model = torch.jit.load(weights)
        device = select_device(device)
        half = device.type != 'cpu'  # half precision only supported on CUDA
        model = model.to(device)

        if half:
            model.half()  # to FP16  
        model.eval()
        self.model = model
        self.device = device
        #self.save_dir = '/home/eva/Desktop/Sefa_deneme/simulation_image'
        #os.makedirs(self.save_dir, exist_ok=True)
        #self.latest_image = None
        #self.timer = self.create_timer(0.01, self.save_latest_image)  # 0.25 saniyede bir çalışır

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.detect(self.latest_image)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    #def save_latest_image(self):
    #    if self.latest_image is not None:
    #        image_path = os.path.join(self.save_dir, 'image.jpg')
    #        cv2.imwrite(image_path, self.latest_image)
    #        self.get_logger().info(f'Image saved: {image_path}')
    #        
    #        # Call the detection function after saving the image
    #        self.detect(image_path)

    def detect(self, source, imgsz=640, conf_thres=0.3, iou_thres=0.45, 
               device='0', classes=None, agnostic_nms=False,):
        # setting and directories
        # save_img = not nosave and not source.endswith('.txt')  # save inference images
        # save_dir = Path(increment_path(Path(project) / name, exist_ok=exist_ok))  # increment run
        # (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

        # inf_time = AverageMeter()
        # waste_time = AverageMeter()
        # nms_time = AverageMeter()

        # Load model
        stride = 32
        # model = torch.jit.load(weights)
        # device = select_device(device)
        # half = device.type != 'cpu'  # half precision only supported on CUDA
        # model = model.to(device)

        # if half:
        #     model.half()  # to FP16  
        # model.eval()

        # Set Dataloader
        #vid_path, vid_writer = None, None
        #dataset = LoadImages(source, img_size=imgsz, stride=stride)
        ###############################################################################################
        model = self.model
        device = self.device
        half = device.type != 'cpu'
        img0 = cv2.resize(source, (1280,720), interpolation=cv2.INTER_LINEAR)
        img = letterbox(img0, imgsz, stride=stride)[0]
        
        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
        t0 = time.time()
        #for path, img, im0s in dataset:
        im0s = img0
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        # Inference
        with torch.no_grad():
            t1 = time_synchronized()
            [pred, anchor_grid], seg, ll = model(img)
            t2 = time_synchronized()

        # waste time: the incompatibility of torch.jit.trace causes extra time consumption in demo version 
        # but this problem will not appear in official version 
        tw1 = time_synchronized()
        pred = split_for_trace_model(pred, anchor_grid)
        tw2 = time_synchronized()

        # Apply NMS
        t3 = time_synchronized()
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)
        t4 = time_synchronized()

        da_seg_mask = driving_area_mask(seg)
        ll_seg_mask = lane_line_mask(ll)

            # Process detections
        for i, det in enumerate(pred):  # detections per image
            #p, s, im0, frame = path, '', im0s, getattr(dataset, 'frame', 0)
            #p = Path(p)  # to Path
            #save_path = str(save_dir / p.name)  # img.jpg
            #txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
            #s += '%gx%g ' % img.shape[2:]  # print string
            #gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0s.shape).round()
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    # if save_txt:  # Write to file
                    #     xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    #     line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                    #     with open(txt_path + '.txt', 'a') as f:
                    #         f.write(('%g ' * len(line)).rstrip() % line + '\n')
                    # if save_img:  # Add bbox to image
                    plot_one_box(xyxy, im0s, line_thickness=3)
            # Show result
            show_seg_result(im0s, (da_seg_mask, ll_seg_mask), is_demo=True)
            # Save results
            # if save_img:
            #     cv2.imwrite(save_path, im0)
            #     print(f" The image with the result is saved in: {save_path}")
            cv2.imshow("hasan",im0s)
            cv2.waitKey(1)
        print(f'Done. ({time.time() - t0:.3f}s)')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
