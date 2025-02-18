import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
from pathlib import Path
import torch
from evata_sim.utils.utils import \
    time_synchronized, select_device, increment_path, \
    scale_coords, xyxy2xywh, non_max_suppression, split_for_trace_model, \
    driving_area_mask, lane_line_mask, plot_one_box, show_seg_result, \
    AverageMeter, LoadImages

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.save_dir = '/home/eva/Desktop/Sefa_deneme/simulation_image'
        os.makedirs(self.save_dir, exist_ok=True)
        self.latest_image = None
        self.timer = self.create_timer(0.01, self.save_latest_image)  # 0.25 saniyede bir çalışır

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def save_latest_image(self):
        if self.latest_image is not None:
            image_path = os.path.join(self.save_dir, 'image.jpg')
            cv2.imwrite(image_path, self.latest_image)
            self.get_logger().info(f'Image saved: {image_path}')
            
            # Call the detection function after saving the image
            self.detect(image_path)

    def detect(self, source, weights='/home/eva/Desktop/Sefa_deneme/yolopv2.pt', 
               save_txt=False, imgsz=640, conf_thres=0.3, iou_thres=0.45, 
               device='0', save_conf=False, nosave=True, classes=None, 
               agnostic_nms=False, project='runs/detect', name='exp', 
               exist_ok=False):
        # setting and directories
        save_img = not nosave and not source.endswith('.txt')  # save inference images
        save_dir = Path(increment_path(Path(project) / name, exist_ok=exist_ok))  # increment run
        (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

        inf_time = AverageMeter()
        waste_time = AverageMeter()
        nms_time = AverageMeter()

        # Load model
        stride = 32
        model = torch.jit.load(weights)
        device = select_device(device)
        half = device.type != 'cpu'  # half precision only supported on CUDA
        model = model.to(device)

        if half:
            model.half()  # to FP16  
        model.eval()

        # Set Dataloader
        vid_path, vid_writer = None, None
        dataset = LoadImages(source, img_size=imgsz, stride=stride)

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
        t0 = time.time()
        for path, img, im0s, vid_cap in dataset:
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
                p, s, im0, frame = path, '', im0s, getattr(dataset, 'frame', 0)

                p = Path(p)  # to Path
                save_path = str(save_dir / p.name)  # img.jpg
                txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
                s += '%gx%g ' % img.shape[2:]  # print string
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                        if save_txt:  # Write to file
                            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                            line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                            with open(txt_path + '.txt', 'a') as f:
                                f.write(('%g ' * len(line)).rstrip() % line + '\n')

                        if save_img:  # Add bbox to image
                            plot_one_box(xyxy, im0, line_thickness=3)

                # Show result
                show_seg_result(im0, (da_seg_mask, ll_seg_mask), is_demo=True)

                # Save results
                if save_img:
                    cv2.imwrite(save_path, im0)
                    print(f" The image with the result is saved in: {save_path}")
                cv2.imshow("hasan",im0)
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