import os
import cv2
import numpy as np
from uav_car_unit.coco_utils import COCO_test_helper
from uav_car_unit.rknn_executor import RKNN_model_container
# from py_utils.pytorch_executor import Torch_model_container
# from py_utils.onnx_executor import ONNX_model_container


class YOLODetector:
    """
    YOLO检测器类，支持多种模型格式（RKNN、PyTorch、ONNX）
    """
    
    def __init__(self, model_path, classes=None, obj_thresh=0.25, nms_thresh=0.45, img_size=(640, 640)):
        """
        初始化YOLO检测器
        
        Args:
            model_path (str): 模型文件路径
            classes (list): 类别名称列表，默认为动物类别
            obj_thresh (float): 目标检测阈值
            nms_thresh (float): NMS阈值
            img_size (tuple): 输入图像尺寸 (width, height)
        """
        self.model_path = model_path
        self.obj_thresh = obj_thresh
        self.nms_thresh = nms_thresh
        self.img_size = img_size
        
        # 默认类别
        self.classes = classes if classes is not None else  ["monkey", "tiger", "elephant", "wolf", "peacock"]
        
        # 初始化模型
        self.model, self.platform = self._setup_model()
        
        # 初始化图像处理工具
        self.co_helper = COCO_test_helper(enable_letter_box=True)
        
        print(f"YOLODetector initialized with {self.platform} model")
        print(f"Classes: {self.classes}")
        print(f"Thresholds - OBJ: {self.obj_thresh}, NMS: {self.nms_thresh}")
    
    def _setup_model(self):
        """设置模型"""
        if self.model_path.endswith('.pt') or self.model_path.endswith('.torchscript'):
            platform = 'pytorch'
            model = Torch_model_container(self.model_path)
        elif self.model_path.endswith('.rknn'):
            platform = 'rknn'
            model = RKNN_model_container(self.model_path)
        elif self.model_path.endswith('.onnx'):
            platform = 'onnx'
            model = ONNX_model_container(self.model_path)
        else:
            raise ValueError(f"{self.model_path} is not a supported model format (pt/torchscript/rknn/onnx)")
        
        return model, platform
    
    def _dfl(self, position):
        """Distribution Focal Loss (DFL)"""
        try:
            import torch
            x = torch.tensor(position)
            n, c, h, w = x.shape
            p_num = 4
            mc = c // p_num
            y = x.reshape(n, p_num, mc, h, w)
            y = y.softmax(2)
            acc_metrix = torch.tensor(range(mc)).float().reshape(1, 1, mc, 1, 1)
            y = (y * acc_metrix).sum(2)
            return y.numpy()
        except ImportError:
            # 如果没有torch，使用numpy实现简化版本
            return position
    
    def _box_process(self, position):
        """处理检测框"""
        grid_h, grid_w = position.shape[2:4]
        col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
        col = col.reshape(1, 1, grid_h, grid_w)
        row = row.reshape(1, 1, grid_h, grid_w)
        grid = np.concatenate((col, row), axis=1)
        stride = np.array([self.img_size[1]//grid_h, self.img_size[0]//grid_w]).reshape(1, 2, 1, 1)

        position = self._dfl(position)
        box_xy = grid + 0.5 - position[:, 0:2, :, :]
        box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
        xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)

        return xyxy
    
    def _filter_boxes(self, boxes, box_confidences, box_class_probs):
        """根据阈值过滤检测框"""
        box_confidences = box_confidences.reshape(-1)
        candidate, class_num = box_class_probs.shape

        class_max_score = np.max(box_class_probs, axis=-1)
        classes = np.argmax(box_class_probs, axis=-1)

        _class_pos = np.where(class_max_score * box_confidences >= self.obj_thresh)
        scores = (class_max_score * box_confidences)[_class_pos]

        boxes = boxes[_class_pos]
        classes = classes[_class_pos]

        return boxes, classes, scores
    
    def _nms_boxes(self, boxes, scores):
        """非极大值抑制"""
        x = boxes[:, 0]
        y = boxes[:, 1]
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]

        areas = w * h
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x[i], x[order[1:]])
            yy1 = np.maximum(y[i], y[order[1:]])
            xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
            yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

            w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
            h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
            inter = w1 * h1

            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(ovr <= self.nms_thresh)[0]
            order = order[inds + 1]
        
        return np.array(keep)
    
    def _post_process(self, input_data):
        """后处理检测结果"""
        boxes, scores, classes_conf = [], [], []
        defualt_branch = 3
        pair_per_branch = len(input_data) // defualt_branch
        
        # 处理每个分支的输出
        for i in range(defualt_branch):
            boxes.append(self._box_process(input_data[pair_per_branch * i]))
            classes_conf.append(input_data[pair_per_branch * i + 1])
            scores.append(np.ones_like(input_data[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32))

        def sp_flatten(_in):
            ch = _in.shape[1]
            _in = _in.transpose(0, 2, 3, 1)
            return _in.reshape(-1, ch)

        boxes = [sp_flatten(_v) for _v in boxes]
        classes_conf = [sp_flatten(_v) for _v in classes_conf]
        scores = [sp_flatten(_v) for _v in scores]

        boxes = np.concatenate(boxes)
        classes_conf = np.concatenate(classes_conf)
        scores = np.concatenate(scores)

        # 过滤检测框
        boxes, classes, scores = self._filter_boxes(boxes, scores, classes_conf)

        # NMS
        nboxes, nclasses, nscores = [], [], []
        for c in set(classes):
            inds = np.where(classes == c)
            b = boxes[inds]
            c_cls = classes[inds]
            s = scores[inds]
            keep = self._nms_boxes(b, s)

            if len(keep) != 0:
                nboxes.append(b[keep])
                nclasses.append(c_cls[keep])
                nscores.append(s[keep])

        if not nclasses and not nscores:
            return None, None, None

        boxes = np.concatenate(nboxes)
        classes = np.concatenate(nclasses)
        scores = np.concatenate(nscores)

        return boxes, classes, scores
    
    def preprocess_image(self, image):
        """
        图像预处理
        
        Args:
            image (np.ndarray): 输入图像
            
        Returns:
            np.ndarray: 预处理后的图像
        """
        # 使用letter box进行尺寸调整
        processed_img = self.co_helper.letter_box(
            im=image.copy(), 
            new_shape=(self.img_size[1], self.img_size[0]), 
            pad_color=(0, 0, 0)
        )
        
        # 根据不同平台处理数据格式
        if self.platform == 'pytorch':
            # PyTorch需要RGB格式和CHW维度顺序
            processed_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2RGB)
            processed_img = processed_img.transpose(2, 0, 1)  # HWC -> CHW
        elif self.platform == 'rknn':
            # RKNN需要BGR格式和HWC维度顺序（保持不变）
            # processed_img保持HWC格式，不进行transpose
            pass
        elif self.platform == 'onnx':
            # ONNX通常需要RGB格式和CHW维度顺序
            processed_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2RGB)
            processed_img = processed_img.transpose(2, 0, 1)  # HWC -> CHW
        
        # 添加batch维度
        processed_img = np.expand_dims(processed_img, 0)  # 添加batch维度
        
        return processed_img
    
    def detect(self, image):
        """
        检测图像中的目标
        
        Args:
            image (np.ndarray): 输入图像
            
        Returns:
            dict: 检测结果，包含boxes, classes, scores, class_names
        """
        # 预处理
        processed_img = self.preprocess_image(image)
        
        # 推理
        outputs = self.model.run([processed_img])
        
        # 后处理
        boxes, classes, scores = self._post_process(outputs)
        
        if boxes is None:
            return {
                'boxes': np.array([]),
                'classes': np.array([]),
                'scores': np.array([]),
                'class_names': []
            }
        
        # 转换回原图坐标
        real_boxes = self.co_helper.get_real_box(boxes)
        
        # 获取类别名称
        class_names = []
        for cls_id in classes:
            if cls_id < len(self.classes):
                class_names.append(self.classes[cls_id])
            else:
                class_names.append(f"unknown_{cls_id}")
        
        return {
            'boxes': real_boxes,
            'classes': classes,
            'scores': scores,
            'class_names': class_names
        }
    
    def draw_detections(self, image, detection_result, draw_conf=True):
        """
        在图像上绘制检测结果
        
        Args:
            image (np.ndarray): 输入图像
            detection_result (dict): 检测结果
            draw_conf (bool): 是否绘制置信度
            
        Returns:
            np.ndarray: 绘制了检测结果的图像
        """
        result_img = image.copy()
        boxes = detection_result['boxes']
        scores = detection_result['scores']
        class_names = detection_result['class_names']
        
        for box, score, class_name in zip(boxes, scores, class_names):
            x1, y1, x2, y2 = [int(coord) for coord in box]
            
            # 绘制边界框
            cv2.rectangle(result_img, (x1, y1), (x2, y2), (255, 255, 255), 2)
            
            # 绘制标签
            if draw_conf:
                label = f'{class_name}: {score:.2f}'
            else:
                label = class_name
            
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(result_img, (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), (255, 255, 255), -1)
            cv2.putText(result_img, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        return result_img
    
    def detect_and_draw(self, image, draw_conf=True):
        """
        检测并绘制结果的便捷方法
        
        Args:
            image (np.ndarray): 输入图像
            draw_conf (bool): 是否绘制置信度
            
        Returns:
            tuple: (detection_result, result_image)
        """
        detection_result = self.detect(image)
        result_image = self.draw_detections(image, detection_result, draw_conf)
        return detection_result, result_image
    
    def release(self):
        """释放模型资源"""
        if hasattr(self, 'model'):
            self.model.release()
            print("YOLODetector resources released")
    
    def __del__(self):
        """析构函数"""
        try:
            self.release()
        except:
            pass


# 示例使用函数
def create_yolo_detector(model_path, classes=None):
    """
    创建YOLO检测器的便捷函数
    
    Args:
        model_path (str): 模型路径
        classes (list): 类别列表
        
    Returns:
        YOLODetector: YOLO检测器实例
    """
    return YOLODetector(model_path, classes)


if __name__ == "__main__":
    # 示例使用
    model_path = "/home/orangepi/Project/rknn_model_zoo-main/examples/yolo11/model/ball.rknn"
    
    # 创建检测器
    detector = YOLODetector(model_path)
    
    # 加载测试图像
    test_image = cv2.imread("/path/to/test/image.jpg")
    
    if test_image is not None:
        # 检测
        result = detector.detect(test_image)
        
        # 打印结果
        print(f"检测到 {len(result['boxes'])} 个目标")
        for i, (box, class_name, score) in enumerate(zip(result['boxes'], result['class_names'], result['scores'])):
            print(f"目标 {i+1}: {class_name} @ {box} (置信度: {score:.3f})")
        
        # 绘制结果
        result_img = detector.draw_detections(test_image, result)
        cv2.imwrite("detection_result.jpg", result_img)
    
    # 释放资源
    detector.release()
