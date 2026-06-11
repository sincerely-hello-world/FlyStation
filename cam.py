import cv2
import numpy as np

def nothing(x):
    pass

def test_color_mask():
    # 创建窗口
    cv2.namedWindow('Adjust HSV Thresholds')
    cv2.namedWindow('Original')
    cv2.namedWindow('Mask')
    cv2.namedWindow('Result')
    
    # 创建HSV阈值的滑动条
    # 低H阈值: 0-179 (OpenCV中H的范围是0-179)
    cv2.createTrackbar('Low H', 'Adjust HSV Thresholds', 10, 179, nothing)
    # 高H阈值
    cv2.createTrackbar('High H', 'Adjust HSV Thresholds', 25, 179, nothing)
    # 低S阈值: 0-255
    cv2.createTrackbar('Low S', 'Adjust HSV Thresholds', 90, 255, nothing)
    # 高S阈值
    cv2.createTrackbar('High S', 'Adjust HSV Thresholds', 255, 255, nothing)
    # 低V阈值: 0-255
    cv2.createTrackbar('Low V', 'Adjust HSV Thresholds', 90, 255, nothing)
    # 高V阈值
    cv2.createTrackbar('High V', 'Adjust HSV Thresholds', 255, 255, nothing)
    
    # 可以选择使用摄像头或图片
    use_camera = input("是否使用摄像头？(y/n): ").lower() == 'y'
    
    if use_camera:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("无法打开摄像头")
            return
        print("按 'q' 退出，按 's' 保存当前HSV阈值")
    else:
        # 使用测试图片，你可以替换成自己的图片路径
        img_path = input("请输入图片路径（直接回车使用默认测试图片）: ").strip()
        if not img_path:
            # 创建一个测试图片
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            # 添加一些橙色色块
            img[100:200, 100:200] = (0, 165, 255)  # 橙色 BGR
            img[200:300, 300:400] = (0, 100, 255)  # 深橙色
            img[300:400, 100:200] = (0, 200, 200)  # 浅橙色
            img[100:200, 400:500] = (0, 0, 255)    # 红色
            img[200:300, 100:200] = (0, 255, 0)    # 绿色
            img[300:400, 400:500] = (255, 0, 0)    # 蓝色
            print("使用内置测试图片")
        else:
            img = cv2.imread(img_path)
            if img is None:
                print("无法读取图片，使用内置测试图片")
                img = np.zeros((480, 640, 3), dtype=np.uint8)
                img[100:200, 100:200] = (0, 165, 255)
    
    print("\n调整滑动条来查看掩膜效果")
    print("橙色通常范围: H: 10-25, S: 100-255, V: 100-255")
    print("按 'q' 退出，按 'r' 重置默认值")
    
    while True:
        if use_camera:
            ret, frame = cap.read()
            if not ret:
                print("无法读取摄像头画面")
                break
            img = frame
        
        # 获取当前滑动条的值
        low_h = cv2.getTrackbarPos('Low H', 'Adjust HSV Thresholds')
        high_h = cv2.getTrackbarPos('High H', 'Adjust HSV Thresholds')
        low_s = cv2.getTrackbarPos('Low S', 'Adjust HSV Thresholds')
        high_s = cv2.getTrackbarPos('High S', 'Adjust HSV Thresholds')
        low_v = cv2.getTrackbarPos('Low V', 'Adjust HSV Thresholds')
        high_v = cv2.getTrackbarPos('High V', 'Adjust HSV Thresholds')
        
        # 确保低阈值不大于高阈值
        if low_h > high_h:
            low_h, high_h = high_h, low_h
            cv2.setTrackbarPos('Low H', 'Adjust HSV Thresholds', low_h)
            cv2.setTrackbarPos('High H', 'Adjust HSV Thresholds', high_h)
        if low_s > high_s:
            low_s, high_s = high_s, low_s
            cv2.setTrackbarPos('Low S', 'Adjust HSV Thresholds', low_s)
            cv2.setTrackbarPos('High S', 'Adjust HSV Thresholds', high_s)
        if low_v > high_v:
            low_v, high_v = high_v, low_v
            cv2.setTrackbarPos('Low V', 'Adjust HSV Thresholds', low_v)
            cv2.setTrackbarPos('High V', 'Adjust HSV Thresholds', high_v)
        
        # 创建阈值数组
        lower = np.array([low_h, low_s, low_v])
        upper = np.array([high_h, high_s, high_v])
        
        # 转换到HSV空间并创建掩膜
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        
        # 应用掩膜到原图
        result = cv2.bitwise_and(img, img, mask=mask)
        
        # 显示当前阈值
        cv2.putText(img, f"H: [{low_h}-{high_h}]", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, f"S: [{low_s}-{high_s}]", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, f"V: [{low_v}-{high_v}]", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 显示图像
        cv2.imshow('Original', img)
        cv2.imshow('Mask', mask)
        cv2.imshow('Result', result)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            # 重置为默认橙色阈值
            cv2.setTrackbarPos('Low H', 'Adjust HSV Thresholds', 10)
            cv2.setTrackbarPos('High H', 'Adjust HSV Thresholds', 25)
            cv2.setTrackbarPos('Low S', 'Adjust HSV Thresholds', 100)
            cv2.setTrackbarPos('High S', 'Adjust HSV Thresholds', 255)
            cv2.setTrackbarPos('Low V', 'Adjust HSV Thresholds', 100)
            cv2.setTrackbarPos('High V', 'Adjust HSV Thresholds', 255)
            print("重置为默认橙色阈值")
        elif key == ord('s'):
            print(f"\n当前HSV阈值:")
            print(f"lower_{'orange'} = np.array([{low_h}, {low_s}, {low_v}])")
            print(f"upper_{'orange'} = np.array([{high_h}, {high_s}, {high_v}])")
    
    if use_camera:
        cap.release()
    cv2.destroyAllWindows()

def test_single_color():
    """测试单个颜色的HSV值"""
    print("\n=== 测试单个像素的HSV值 ===")
    print("这个函数可以帮助你确定某个颜色的HSV范围")
    
    # 创建测试图像
    img = np.zeros((300, 600, 3), dtype=np.uint8)
    
    # 创建HSV显示窗口
    cv2.namedWindow('Color Tester')
    
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            # 获取鼠标位置的BGR颜色
            bgr_color = img[y, x]
            # 转换为HSV
            hsv_color = cv2.cvtColor(np.uint8([[bgr_color]]), cv2.COLOR_BGR2HSV)[0][0]
            
            # 显示颜色信息
            info = f"BGR: {bgr_color} | HSV: {hsv_color}"
            print(f"\r{info}", end='', flush=True)
    
    cv2.setMouseCallback('Color Tester', mouse_callback)
    
    # 创建颜色选择滑动条
    cv2.createTrackbar('B', 'Color Tester', 0, 255, nothing)
    cv2.createTrackbar('G', 'Color Tester', 0, 255, nothing)
    cv2.createTrackbar('R', 'Color Tester', 0, 255, nothing)
    
    print("移动鼠标查看颜色值，或使用滑动条创建颜色")
    print("按 'q' 退出")
    
    while True:
        # 获取当前BGR值
        b = cv2.getTrackbarPos('B', 'Color Tester')
        g = cv2.getTrackbarPos('G', 'Color Tester')
        r = cv2.getTrackbarPos('R', 'Color Tester')
        
        # 填充图像
        img[:] = [b, g, r]
        
        # 显示当前颜色
        cv2.imshow('Color Tester', img)
        
        key = cv2.waitKey(10) & 0xFF
        if key == ord('q'):
            break
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("请选择测试模式:")
    print("1. 测试颜色掩膜（带滑动条）")
    print("2. 测试单个颜色HSV值（帮助确定阈值）")
    
    choice = input("请输入选择 (1/2): ").strip()
    
    if choice == '1':
        test_color_mask()
    elif choice == '2':
        test_single_color()
    else:
        print("无效选择，运行默认测试")
        test_color_mask()